/*
 * Copyright 2022 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "map_area_filter_contents.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
const std::unordered_map<uint8_t, std::string> object_classification_map_ = {
  {ObjectClassification::UNKNOWN, "unknown"},
  {ObjectClassification::CAR, "car"},
  {ObjectClassification::TRUCK, "truck"},
  {ObjectClassification::BUS, "bus"},
  {ObjectClassification::TRAILER, "trailer"},
  {ObjectClassification::MOTORCYCLE, "motorcycle"},
  {ObjectClassification::PEDESTRIAN, "pedestrian"},
  {ObjectClassification::BICYCLE, "bicycle"}};
}  // namespace

namespace map_area_filter
{
bool RemovalArea::is_in_distance(const geometry_msgs::msg::Point pos, const double distance) const
{
  for (const auto & vertex_point : polygon_) {
    const double dist_to_vertex = std::hypot(vertex_point.x() - pos.x, vertex_point.y() - pos.y);
    if (dist_to_vertex < distance) {
      return true;
    }
  }
  const double dist_to_polygon = boost::geometry::distance(polygon_, PointXY(pos.x, pos.y));
  return (dist_to_polygon < distance);
};

// constructor
MapAreaFilterComponent::MapAreaFilterComponent(const rclcpp::NodeOptions & options)
: Node("MapAreaFilter", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  // set initial parameters
  enable_object_filtering_ =
    static_cast<bool>(this->declare_parameter("enable_object_filtering", false));
  enable_pointcloud_filtering_ =
    static_cast<bool>(this->declare_parameter("enable_pointcloud_filtering", false));
  min_guaranteed_area_distance_ =
    static_cast<double>(this->declare_parameter("min_guaranteed_area_distance", 100.0));
  map_frame_ = static_cast<std::string>(this->declare_parameter("map_frame", "map"));
  base_link_frame_ =
    static_cast<std::string>(this->declare_parameter("base_link_frame", "base_link"));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MapAreaFilterComponent::paramCallback, this, _1));

  if (!enable_object_filtering_ && !enable_pointcloud_filtering_) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Both object filtering and pointcloud filtering are disabled. The map_area_filter node will "
      "not filter anything. Disabling the node is recommended.");
  }

  const int max_queue_size = 5;

  pub_objects_ = this->create_publisher<PredictedObjects>("output/objects", rclcpp::QoS(10));
  pub_pointcloud_ = this->create_publisher<PointCloud2>(
    "output/pointcloud", rclcpp::SensorDataQoS().keep_last(max_queue_size));

  sub_lanelet_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/vector_map", rclcpp::QoS(10).transient_local(),
    std::bind(&MapAreaFilterComponent::lanelet_map_callback, this, _1));
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", rclcpp::QoS(1),
    std::bind(&MapAreaFilterComponent::odometry_callback, this, _1));
  sub_objects_ = this->create_subscription<PredictedObjects>(
    "input/objects", rclcpp::QoS(10),
    std::bind(&MapAreaFilterComponent::objects_callback, this, _1));
  std::function<void(const PointCloud2ConstPtr msg)> cb =
    std::bind(&MapAreaFilterComponent::computePublish, this, _1);
  sub_pointcloud_ = create_subscription<PointCloud2>(
    "input/pointcloud", rclcpp::SensorDataQoS().keep_last(max_queue_size), cb);
}

void MapAreaFilterComponent::computePublish(const PointCloud2ConstPtr & input)
{
  auto output = std::make_unique<PointCloud2>();

  // Call the virtual method in the child
  filter(input, *output);

  // Publish a boost shared ptr
  pub_pointcloud_->publish(std::move(output));
}

void MapAreaFilterComponent::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::scoped_lock lock(mutex_);
  kinematic_state_ = *msg;
}

void MapAreaFilterComponent::lanelet_map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg)
{
  std::scoped_lock lock(mutex_);

  autoware::route_handler::RouteHandler route_handler;
  route_handler.setMap(*msg);
  const auto lanelet_map_ptr = route_handler.getLaneletMapPtr();
  if (!lanelet_map_ptr) {
    return;
  }

  removal_areas_.clear();
  for (const auto & polygon_layer_elemet : lanelet_map_ptr->polygonLayer) {
    std::string type_val = polygon_layer_elemet.attributeOr(lanelet::AttributeName::Type, "");

    if (type_val != "map_area_filter") {
      continue;
    }
    if (polygon_layer_elemet.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Found map_area_filter polygon id: %ld but it has no vertices, skipping.",
        polygon_layer_elemet.id());
      continue;
    }

    RemovalArea removal_area;
    removal_area.id_ = polygon_layer_elemet.id();
    removal_area.polygon_ = lanelet::utils::to2D(polygon_layer_elemet).basicPolygon();
    boost::geometry::correct(removal_area.polygon_);

    removal_area.envelope_box_ = boost::geometry::return_envelope<Box2D>(removal_area.polygon_);

    double sum_z = std::accumulate(
      polygon_layer_elemet.begin(), polygon_layer_elemet.end(), 0.0,
      [](double sum, const auto & point) { return sum + point.z(); });
    removal_area.centroid_height_ = sum_z / static_cast<double>(polygon_layer_elemet.size());

    for (const auto & attribute : polygon_layer_elemet.attributes()) {
      const std::string key = attribute.first;
      const std::string value = attribute.second.value();

      if (key == "remove_below_height") {
        try {
          removal_area.remove_below_height_ = std::stod(value);
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            this->get_logger(),
            "Invalid value type against the key remove_below_height for polygon %ld: %s",
            polygon_layer_elemet.id(), e.what());
        }
      } else if (key == "remove_above_height") {
        try {
          removal_area.remove_above_height_ = std::stod(value);
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            this->get_logger(),
            "Invalid value type against the key remove_above_height for polygon %ld: %s",
            polygon_layer_elemet.id(), e.what());
        }
      } else if (key != "type" && key != "subtype") {
        if (value == "remove") {
          removal_area.target_labels_.insert(key);
        }
      }
    }

    if (!removal_area.target_labels_.empty()) {
      removal_areas_.push_back(removal_area);
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "map_area_filter: %lu removal areas are registered", removal_areas_.size());
  if (removal_areas_.empty()) {
    RCLCPP_WARN(this->get_logger(), "map_area_filter: No removal areas are found in the map.");
  }
}

void MapAreaFilterComponent::objects_callback(const PredictedObjects::ConstSharedPtr & msg)
{
  std::scoped_lock lock(mutex_);
  objects_ptr_ = msg;

  PredictedObjects out_objects;

  if (!enable_object_filtering_ || removal_areas_.empty()) {
    pub_objects_->publish(*objects_ptr_.get());
  } else if (filter_objects_by_area(out_objects)) {
    pub_objects_->publish(out_objects);
  }
}

bool MapAreaFilterComponent::transform_pointcloud(  // output flame
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output)
{
  try {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped = tf2.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp, rclcpp::Duration::from_seconds(0.5));

    Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(tf_matrix, input, output);
    output.header.stamp = input.header.stamp;
    output.header.frame_id = target_frame;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("map_area_filter"), ex.what());
    return false;
  }
  return true;
}

void MapAreaFilterComponent::filter_points_by_area(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
  // (v^2) / (2*a) + min_distance
  constexpr double a = 1.0;  // m/s^2
  const double attention_length =
    std::pow(kinematic_state_.twist.twist.linear.x, 2) / (2.0 * a) + min_guaranteed_area_distance_;

  const auto point_size = input->points.size();  // subscribeされた点群のtopic
  std::vector<bool> within(point_size, false);

  std::vector<RemovalArea> active_removal_areas;
  for (const auto & removal_area : removal_areas_) {
    if (!removal_area.is_in_distance(kinematic_state_.pose.pose.position, attention_length)) {
      continue;
    }

    if (
      removal_area.target_labels_.count("pointcloud") == 0 &&
      removal_area.target_labels_.count("all") == 0) {
      continue;
    }
    active_removal_areas.push_back(removal_area);
  }

#pragma omp parallel for num_threads(1)
  for (std::size_t point_i = 0; point_i < point_size; ++point_i) {
    const auto & point = input->points[point_i];
    for (const auto & removal_area : active_removal_areas) {
      const auto & remove_above = removal_area.remove_above_height_;
      const auto & remove_bellow = removal_area.remove_below_height_;
      const double point_height = point.z - removal_area.centroid_height_;
      if (remove_above.has_value() || remove_bellow.has_value()) {
        const bool hit_above = remove_above.has_value() && (point_height > remove_above.value());
        const bool hit_bellow = remove_bellow.has_value() && (point_height < remove_bellow.value());
        if (!hit_above && !hit_bellow) {
          continue;
        }
      }

      const auto point_xy = PointXY(point.x, point.y);
      if (
        boost::geometry::within(point_xy, removal_area.envelope_box_) &&
        boost::geometry::within(point_xy, removal_area.polygon_)) {
        within[point_i] = true;
        break;
      }
    }
  }

  for (std::size_t point_i = 0; point_i < point_size; ++point_i) {
    if (!within[point_i]) {
      const auto point = input->points[point_i];
      output->points.emplace_back(pcl::PointXYZ(point.x, point.y, point.z));
      // todo (takagi): restrict copy
    }
  }
}

bool MapAreaFilterComponent::filter_objects_by_area(PredictedObjects & out_objects)
{
  if (!objects_ptr_.has_value() || objects_ptr_.get() == nullptr) {
    return false;
  }  // objects_ptr_はinput_objectsのpointer

  const PredictedObjects in_objects = *objects_ptr_.get();  // input_objectのデータ
  out_objects.header = in_objects.header;
  assert(in_objects.header.frame_id == "map");

  // (v^2) / (2*a) + min_distance
  constexpr double a = 1.0;  // m/s^2
  const double attention_length =
    std::pow(kinematic_state_.twist.twist.linear.x, 2) / (2.0 * a) + min_guaranteed_area_distance_;
  std::vector<RemovalArea> active_removal_areas;
  for (const auto & removal_area : removal_areas_) {
    if (removal_area.is_in_distance(kinematic_state_.pose.pose.position, attention_length)) {
      active_removal_areas.push_back(removal_area);
    }
  }

  for (const auto & object : in_objects.objects) {
    bool within = false;
    for (const auto & removal_area : active_removal_areas) {
      if (
        removal_area.target_labels_.count(
          object_classification_map_.at(object.classification[0].label)) == 0 &&
        removal_area.target_labels_.count("all") == 0 &&
        removal_area.target_labels_.count("all_objects") == 0) {
        continue;
      }

      const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
      const auto & remove_above = removal_area.remove_above_height_;
      const auto & remove_bellow = removal_area.remove_below_height_;
      const double lower_height =
        pos.z - object.shape.dimensions.z * 0.5 - removal_area.centroid_height_;
      const double upper_height =
        pos.z + object.shape.dimensions.z * 0.5 - removal_area.centroid_height_;
      if (remove_above.has_value() || remove_bellow.has_value()) {
        const bool hit_above = remove_above.has_value() && (lower_height > remove_above.value());
        const bool hit_bellow = remove_bellow.has_value() && (upper_height < remove_bellow.value());
        if (!hit_above && !hit_bellow) {
          continue;
        }
      }

      const auto obj_centroid_xy = PointXY(pos.x, pos.y);
      if (
        boost::geometry::within(obj_centroid_xy, removal_area.envelope_box_) &&
        boost::geometry::within(obj_centroid_xy, removal_area.polygon_)) {
        within = true;
        break;
      }
    }
    if (!within) {
      out_objects.objects.emplace_back(object);
    }
  }

  objects_ptr_ = boost::none;
  return true;
}

void MapAreaFilterComponent::filter(const PointCloud2ConstPtr & input, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);

  if (!enable_pointcloud_filtering_ || removal_areas_.empty()) {
    output = *input;
    return;
  }

  // transform cloud to filter to map frame
  sensor_msgs::msg::PointCloud2 map_frame_cloud;
  if (!transform_pointcloud(*input, tf_buffer_, map_frame_, map_frame_cloud)) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "Cannot transform cloud to " << map_frame_ << " not filtering.");
    output = *input;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_frame_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(map_frame_cloud, *map_frame_cloud_pcl);

  // create filtered cloud container
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_frame_cloud_pcl_filtered(
    new pcl::PointCloud<pcl::PointXYZ>);
  filter_points_by_area(map_frame_cloud_pcl, map_frame_cloud_pcl_filtered);

  pcl::toROSMsg(*map_frame_cloud_pcl_filtered, output);
  output.header = input->header;
  output.header.frame_id = map_frame_;

  if (!transform_pointcloud(output, tf_buffer_, input->header.frame_id, output)) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "Cannot transform back cloud to " << input->header.frame_id << " not filtering.");
    output = *input;
    return;
  }
  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult MapAreaFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "enable_object_filtering", enable_object_filtering_)) {
    RCLCPP_DEBUG(
      this->get_logger(), "Setting new object filtering to: %s.",
      enable_object_filtering_ ? "true" : "false");
  }
  if (get_param(p, "enable_pointcloud_filtering", enable_pointcloud_filtering_)) {
    RCLCPP_DEBUG(
      this->get_logger(), "Setting new pointcloud filtering to: %s.",
      enable_pointcloud_filtering_ ? "true" : "false");
  }
  if (get_param(p, "min_guaranteed_area_distance", min_guaranteed_area_distance_)) {
    RCLCPP_DEBUG(
      this->get_logger(), "Setting new area check distance to: %.2lf.",
      min_guaranteed_area_distance_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace map_area_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_area_filter::MapAreaFilterComponent);
