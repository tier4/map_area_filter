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

using std::placeholders::_1;
using namespace std::chrono_literals;

void RemovalArea::update_is_in_distance(geometry_msgs::msg::Point pos, double distance)
{
  for (const auto & vertex_point : polygon_) {
    const double dist_to_vertex = std::hypot(vertex_point.x() - pos.x, vertex_point.y() - pos.y);
    if (dist_to_vertex < distance) {
      is_in_distance_ = true;
      return;
    }
  }
  const double dist_to_polygon = boost::geometry::distance(polygon_, PointXY(pos.x, pos.y));
  is_in_distance_ = (dist_to_polygon < distance);
};

// constructor
MapAreaFilterComponent::MapAreaFilterComponent(const rclcpp::NodeOptions & options)
: Filter("MapAreaFilter", options)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  // set initial parameters

  map_frame_ = static_cast<std::string>(this->declare_parameter("map_frame", "map"));
  base_link_frame_ =
    static_cast<std::string>(this->declare_parameter("base_link_frame", "base_link"));
  min_guaranteed_area_distance_ =
    static_cast<double>(this->declare_parameter("min_guaranteed_area_distance", 100));
  marker_font_scale_ = static_cast<double>(this->declare_parameter("marker_font_scale", 1.0));
  // 1: pointcloud_filter 2: object_filter
  filter_type = static_cast<double>(this->declare_parameter("filter_type", 1));
  // todo (takagi): cleanup filter_type usage (combine with do_filter_,
  // 常にトピックの受信は行うなど)

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", rclcpp::QoS(1),
    std::bind(&MapAreaFilterComponent::odometry_callback, this, _1));
  lanelet_map_sub_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/vector_map", rclcpp::QoS(10).transient_local(),
    std::bind(&MapAreaFilterComponent::lanelet_map_callback, this, _1));

  if (filter_type == 1 || filter_type == 3) {
    objects_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/additional_objects_cloud", rclcpp::SensorDataQoS(),
      std::bind(&MapAreaFilterComponent::objects_cloud_callback, this, _1));
    area_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "pointcloud_filter_area", rclcpp::QoS(1));
  }
  if (filter_type == 2 || filter_type == 3) {
    objects_sub_ = this->create_subscription<PredictedObjects>(
      "input/objects", rclcpp::QoS(10),
      std::bind(&MapAreaFilterComponent::objects_callback, this, _1));
    filtered_objects_pub_ =
      this->create_publisher<PredictedObjects>("output/objects", rclcpp::QoS(10));
    area_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "objects_filter_area", rclcpp::QoS(1));
  }
  if (filter_type != 1 && filter_type != 2 && filter_type != 3) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "\nfilter_type: " << filter_type
                        << " is not invalid.\nIn order to filter pointcloud: filter_type = 1\nIn "
                           "order to filter objects: filter_type = 2");
  }

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MapAreaFilterComponent::paramCallback, this, _1));

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_.reset(new tf2_ros::Buffer(clock));
  tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

  timer_ = this->create_wall_timer(1s, std::bind(&MapAreaFilterComponent::timer_callback, this));
}

void MapAreaFilterComponent::color_func(double dis, std_msgs::msg::ColorRGBA & color)
{
  if (filter_type == 2) {
    color.r = 0. + dis;
    color.g = 0. + dis;
    color.b = 1.;
    color.a = 0.5;
  } else if (filter_type == 1) {
    color.r = 1.;
    color.g = 0. + dis;
    color.b = 0. + dis;
    color.a = 0.5;
  } else if (filter_type == 3) {
    color.r = 1.;
    color.g = 0. + dis;
    color.b = 1.;
    color.a = 0.5;
  }
}

void MapAreaFilterComponent::create_area_marker_msg()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(0.0, 0.5);

  area_markers_msg_.markers.clear();

  for (size_t i = 0; i < removal_areas_.size(); i++) {
    visualization_msgs::msg::Marker area;
    area.ns = "polygon";
    area.header.frame_id = map_frame_;
    area.id = i;
    area.type = visualization_msgs::msg::Marker::LINE_STRIP;
    area.action = visualization_msgs::msg::Marker::MODIFY;

    std_msgs::msg::ColorRGBA color;
    color_func(dist(gen), color);

    area.color = color;
    area.scale.x = 0.1;
    area.scale.y = 0.1;
    area.scale.z = 0.1;

    for (const auto & vertex : removal_areas_[i].polygon_) {
      geometry_msgs::msg::Point point;
      point.x = vertex.x();
      point.y = vertex.y();
      area.points.emplace_back(point);
    }
    area.points.emplace_back(area.points.front());
    area_markers_msg_.markers.emplace_back(area);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = map_frame_;
    text.ns = "id";
    text.id = removal_areas_.size() + i;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;

    text.pose.position.x = static_cast<double>(area.points[0].x);
    text.pose.position.y = static_cast<double>(area.points[0].y);
    text.pose.position.z = 0.0;
    text.scale.z = marker_font_scale_;
    text.text = "map_area_filter_removal_area";
    text.color = color;
    area_markers_msg_.markers.emplace_back(text);
  }
}

void MapAreaFilterComponent::timer_callback()
{
  area_markers_pub_->publish(area_markers_msg_);
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

  route_handler_.setMap(*msg);
  const auto lanelet_map_ptr = route_handler_.getLaneletMapPtr();

  removal_areas_.clear();

  if (!lanelet_map_ptr) {
    return;
  }

  for (const auto & polygon_layer_elemet : lanelet_map_ptr->polygonLayer) {
    std::string type_val = polygon_layer_elemet.attributeOr(lanelet::AttributeName::Type, "");

    if (type_val != "map_area_filter") {
      continue;
    }

    RemovalArea removal_area;
    removal_area.id_ = polygon_layer_elemet.id();
    removal_area.polygon_ = lanelet::utils::to2D(polygon_layer_elemet).basicPolygon();
    boost::geometry::correct(removal_area.polygon_);

    for (const auto & attribute : polygon_layer_elemet.attributes()) {
      const std::string key = attribute.first;
      const std::string value = attribute.second.value();

      if (key == "max_removal_height") {
        try {
          removal_area.max_removal_height_ = std::stod(value);
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            this->get_logger(),
            "Invalid value type against the key max_removal_height for polygon %ld: %s",
            polygon_layer_elemet.id(), e.what());
        }
      } else if (key == "min_removal_height") {
        try {
          removal_area.min_removal_height_ = std::stod(value);
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            this->get_logger(),
            "Invalid value type against the key min_removal_height for polygon %ld: %s",
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

  RCLCPP_INFO(this->get_logger(), "%lu removal areas are registered", removal_areas_.size());
  create_area_marker_msg();
}

void MapAreaFilterComponent::objects_cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  std::scoped_lock lock(mutex_);
  objects_cloud_ptr_ = msg;
}

void MapAreaFilterComponent::objects_callback(const PredictedObjects::ConstSharedPtr & msg)
{
  std::scoped_lock lock(mutex_);
  objects_ptr_ = msg;

  PredictedObjects out_objects;
  if (!do_filter_ || removal_areas_.empty()) {
    filtered_objects_pub_->publish(*objects_ptr_.get());
  } else if (filter_objects_by_area(out_objects)) {
    filtered_objects_pub_->publish(out_objects);
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

  for (auto & removal_area : removal_areas_) {
    removal_area.update_is_in_distance(kinematic_state_.pose.pose.position, attention_length);
  }

  const auto point_size = input->points.size();  // subscribeされた点群のtopic
  std::vector<bool> within(point_size, false);
#pragma omp parallel for num_threads(1)
  for (std::size_t point_i = 0; point_i < point_size; ++point_i) {
    const auto & point = input->points[point_i];
    for (const auto & removal_area : removal_areas_) {
      if (!removal_area.is_in_distance_) {
        continue;
      }

      if (
        removal_area.target_labels_.count("pointcloud") == 0 &&
        removal_area.target_labels_.count("all") == 0) {
        continue;
      }

      if (boost::geometry::within(PointXY(point.x, point.y), removal_area.polygon_)) {
        // todo (takagi): height check
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

  for (auto & removal_area : removal_areas_) {
    removal_area.update_is_in_distance(kinematic_state_.pose.pose.position, attention_length);
  }

  for (const auto & object : in_objects.objects) {
    bool within = false;

    for (const auto & removal_area : removal_areas_) {
      if (!removal_area.is_in_distance_) {
        continue;
      }

      if (
        removal_area.target_labels_.count(
          object_classification_map_.at(object.classification[0].label)) == 0 &&
        removal_area.target_labels_.count("all") == 0 &&
        removal_area.target_labels_.count("all_objects") == 0) {
        continue;
      }

      const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
      if (boost::geometry::within(PointXY(pos.x, pos.y), removal_area.polygon_)) {
        // todo (takagi): height check
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

void MapAreaFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);

  if (!do_filter_ || removal_areas_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Not filtering: Check empty areas or Parameters.");
    output = *input;
    return;
  }

  // transform cloud to filter to map frame
  sensor_msgs::msg::PointCloud2 map_frame_cloud;
  if (!transform_pointcloud(*input, *tf2_, map_frame_, map_frame_cloud)) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "Cannot transform cloud to " << map_frame_ << " not filtering.");
    output = *input;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_frame_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(map_frame_cloud, *map_frame_cloud_pcl);

  // transform known object cloud to map frame
  sensor_msgs::msg::PointCloud2 objects_frame_cloud;
  if (objects_cloud_ptr_ != nullptr) {
    if (!transform_pointcloud(*objects_cloud_ptr_, *tf2_, map_frame_, objects_frame_cloud)) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1,
        "Cannot transform objects cloud to " << map_frame_
                                             << " not filtering. Not including object cloud");
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr objects_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(objects_frame_cloud, *objects_cloud_pcl);

  // if known object cloud contains points add to the filtered cloud
  // const std::size_t border = map_frame_cloud_pcl->size();
  if (!objects_cloud_pcl->points.empty()) *map_frame_cloud_pcl += *objects_cloud_pcl;

  // create filtered cloud container
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_frame_cloud_pcl_filtered(
    new pcl::PointCloud<pcl::PointXYZ>);
  filter_points_by_area(map_frame_cloud_pcl, map_frame_cloud_pcl_filtered);

  pcl::toROSMsg(*map_frame_cloud_pcl_filtered, output);
  output.header = input->header;
  output.header.frame_id = map_frame_;

  if (!transform_pointcloud(output, *tf2_, input->header.frame_id, output)) {
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

  if (get_param(p, "do_filter", do_filter_)) {
    RCLCPP_DEBUG(
      this->get_logger(), "Setting new whether to apply filter: %s.",
      do_filter_ ? "true" : "false");
  }
  if (get_param(p, "map_frame", map_frame_)) {
    RCLCPP_DEBUG(this->get_logger(), "Setting new map frame to: %s.", map_frame_.c_str());
  }
  if (get_param(p, "base_link_frame", base_link_frame_)) {
    RCLCPP_DEBUG(
      this->get_logger(), "Setting new base link frame to: %s.", base_link_frame_.c_str());
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
