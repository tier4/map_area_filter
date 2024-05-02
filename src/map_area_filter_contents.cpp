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

namespace map_area_filter
{

using std::placeholders::_1;
using namespace std::chrono_literals;

//constructor
MapAreaFilterComponent::MapAreaFilterComponent(const rclcpp::NodeOptions & options)
: Filter("MapAreaFilter", options)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  // set initial parameters
  std::string map_area_csv;

  do_filter_ = static_cast<bool>(this->declare_parameter("do_filter", true));
  map_frame_ = static_cast<std::string>(this->declare_parameter("map_frame", "map"));
  base_link_frame_ =
    static_cast<std::string>(this->declare_parameter("base_link_frame", "base_link"));
  map_area_csv = static_cast<std::string>(this->declare_parameter("map_area_csv", ""));
  area_distance_check_ = static_cast<double>(this->declare_parameter("area_distance_check", 300));
  marker_font_scale_ = static_cast<double>(this->declare_parameter("marker_font_scale", 1.0));
  // 1: pointcloud_filter 2: object_filter
  filter_type = static_cast<double>(this->declare_parameter("filter_type", 1));
  area_labels.reserve(10000);
  
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/pose_topic", rclcpp::QoS(1),
    std::bind(&MapAreaFilterComponent::pose_callback, this, _1));

  if(filter_type == 1){
    objects_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input/objects_cloud", rclcpp::SensorDataQoS(),
      std::bind(&MapAreaFilterComponent::objects_cloud_callback, this, _1));
    area_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("output/debug1", rclcpp::QoS(1));
  }else if(filter_type == 2){
    objects_sub_ = this->create_subscription<PredictedObjects>(
      "input/objects", rclcpp::QoS(10),
      std::bind(&MapAreaFilterComponent::objects_callback, this, _1));
    filtered_objects_pub_ =
      this->create_publisher<PredictedObjects>("output/objects", rclcpp::QoS(10));
    area_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("output/debug2", rclcpp::QoS(1));
  } else{
      RCLCPP_INFO_STREAM(this->get_logger(), "\nfilter_type: " << filter_type 
        << " is not invalid.\nIn order to filter pointcloud: filter_type = 1\nIn order to filter objects: filter_type = 2");
  }
  
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MapAreaFilterComponent::paramCallback, this, _1));

  RCLCPP_INFO_STREAM(this->get_logger(), "Loading CSV: " << map_area_csv);
  if (map_area_csv.empty() || !load_areas_from_csv(map_area_csv)) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "Invalid CSV File provided: '" << map_area_csv << "'. Not filtering");
    do_filter_ = false;
    csv_invalid = false;
  }else{
    RCLCPP_INFO_STREAM(this->get_logger(), "Areas: " << area_polygons_.size());

    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_.reset(new tf2_ros::Buffer(clock));
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

    timer_ = this->create_wall_timer(1s, std::bind(&MapAreaFilterComponent::timer_callback, this));

    do_filter_ = true;
  }
}

void MapAreaFilterComponent::subscribe() { Filter::subscribe(); }

void MapAreaFilterComponent::unsubscribe() { Filter::unsubscribe(); }

void MapAreaFilterComponent::color_func(double dis,std_msgs::msg::ColorRGBA & color)
{
  if(filter_type == 2) {
  color.r = 0. + dis;
  color.g = 0. + dis;
  color.b = 1.;
  color.a = 0.5;
  } else if(filter_type == 1) {
  color.r = 1.;
  color.g = 0. + dis;
  color.b = 0. + dis;
  color.a = 0.5;
  }
}

void MapAreaFilterComponent::create_area_marker_msg()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(0.0, 0.5);

  for (std::size_t i = 0, size = area_polygons_.size(); i < size; i++) {
    visualization_msgs::msg::Marker area;
    area.ns = "polygon";
    area.header.frame_id = map_frame_;
    area.id = i;
    area.type = visualization_msgs::msg::Marker::LINE_STRIP;
    area.action = visualization_msgs::msg::Marker::MODIFY;

    std_msgs::msg::ColorRGBA color;
    color_func(dist(gen),color);

    area.color = color;

    area.scale.x = 0.1;
    area.scale.y = 0.1;
    area.scale.z = 0.1;

    for (auto it = boost::begin(boost::geometry::exterior_ring(area_polygons_[i])),
              end = boost::end(boost::geometry::exterior_ring(area_polygons_[i]));
         it != end; ++it) {
      geometry_msgs::msg::Point point;
      point.x = it->x();
      point.y = it->y();
      area.points.emplace_back(point);
    }
    area_markers_msg_.markers.emplace_back(area);

    visualization_msgs::msg::Marker text;
    text.header.frame_id = map_frame_;
    text.ns = "id";
    text.id = size + i;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;

    text.pose.position.x = static_cast<double>(area.points[0].x);
    text.pose.position.y = static_cast<double>(area.points[0].y);
    text.pose.position.z = 0.0;
    text.scale.z = marker_font_scale_;
    text.text = std::to_string(original_csv_order_[i]+1);
    text.color = color;
    area_markers_msg_.markers.emplace_back(text);
  }
}

void MapAreaFilterComponent::timer_callback() { area_markers_pub_->publish(area_markers_msg_); }

void MapAreaFilterComponent::pose_callback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  std::scoped_lock lock(mutex_);
  current_pose_ = *msg;
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
  if(csv_invalid){
    filtered_objects_pub_->publish(*objects_ptr_.get());
  }else if (filter_objects_by_area(out_objects)) {
    filtered_objects_pub_->publish(out_objects);
  }
}

void MapAreaFilterComponent::csv_row_func(const csv::CSVRow & row,std::deque<csv::CSVRow> & rows,std::size_t row_i)
{
  if(filter_type == 2) {
    rows.emplace_back(row);
    original_csv_order_.emplace_back(row_i);
  } else if(filter_type == 1) {
    rows.emplace_front(row);
    original_csv_order_.emplace_front(row_i);
  }
}

void MapAreaFilterComponent::row_to_rowpoints(const csv::CSVRow & row ,std::vector<PointXY> & row_points,AreaType & areatype, bool & correct_elem)
{
  size_t i = 0, j = -1;
  float current_x = 0.f;
  RCLCPP_INFO_STREAM(this->get_logger(), "a");
  for (csv::CSVField & field : row) {//各列に対して
    areatype = AreaType::DELETE_OBJECT;
    if (i == 0) {  // first column contains type of area.
      if(filter_type == 2) {
        uint8_t label = (uint8_t)field.get<int>(correct_elem);
        if(correct_elem){area_labels.emplace_back(label);}
      }
      i++;
      j++;
      RCLCPP_INFO_STREAM(this->get_logger(), "a" << correct_elem);
      continue;
    }//1つ目の値を取り出す
    float point = field.get<float>(correct_elem);
    if(correct_elem){
      if (j % 2) {//xをもとに(x,y)を取り出す 
        row_points.emplace_back(PointXY(current_x, point));
      } else {
        current_x = point;
      }
    }
    j++;
  }
}

bool MapAreaFilterComponent::load_areas_from_csv(const std::string & file_name)
{
  csv::CSVFormat format;
  format.no_header();
  format.variable_columns(csv::VariableColumnPolicy::KEEP);
  csv::CSVReader reader(file_name,csv_loaded_,format);
  if(!csv_loaded_){return false;}
  std::deque<csv::CSVRow> rows;
  std::size_t row_i = 0;
  for (const csv::CSVRow & row : reader) {
    csv_row_func(row,rows,row_i);//列の順番を並び替えただけ

    row_i++;
  }

  for (const csv::CSVRow & row : rows) {//各行に対して
    std::vector<PointXY> row_points;
    Polygon2D current_polygon;
    AreaType areatype;
    bool correct_elem = true;//csv fileの各要素が正しい数値かどうか(正しくないならfilterしない)
    row_to_rowpoints(row,row_points,areatype,correct_elem);//一列の情報をrow_pointsに格納
    //current_polygonのログ、エラー処理
    if (correct_elem && !row_points.empty()) {
      boost::geometry::assign_points(current_polygon, row_points);
      if (!boost::geometry::is_valid(current_polygon)) {
        boost::geometry::correct(current_polygon);
      }
      RCLCPP_INFO_STREAM(  // Verbose output
        this->get_logger(),
        "Polygon in row: " << boost::geometry::dsv(current_polygon) << " has an area of "
                          << boost::geometry::area(current_polygon) << " and its type is "
                          << static_cast<int>(areatype));
      if (boost::geometry::area(current_polygon) > 0) {
        PointXY centroid(0.f, 0.f);
        boost::geometry::centroid(current_polygon, centroid);
        centroid_polygons_.emplace_back(centroid);//多角形の重心の集合
        area_polygons_.emplace_back(current_polygon);//多角形の集合(areatypeが適切でないものは無視)
      } else {
        RCLCPP_WARN_STREAM(
          this->get_logger(), "Ignoring invalid polygon:" << boost::geometry::dsv(current_polygon));
      }
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), "Invalid point in CSV:" << row.to_json());
    }
  }

  csv_loaded_ = true;
  create_area_marker_msg();//csvの処理がされた多角形に色を付けて図示

  return !area_polygons_.empty();
}

bool MapAreaFilterComponent::transform_pointcloud(//output flame 
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
  const auto polygon_size = area_polygons_.size();

  std::vector<bool> area_check(polygon_size, false);
  for (std::size_t area_i = 0; area_i < polygon_size; area_i++) {
    const auto & centroid = centroid_polygons_[area_i];
    double distance = sqrt(
      (centroid.x() - current_pose_.pose.position.x) *
        (centroid.x() - current_pose_.pose.position.x) +
      (centroid.y() - current_pose_.pose.position.y) *
        (centroid.y() - current_pose_.pose.position.y));

    area_check[area_i] = (distance <= area_distance_check_);
  }  // for polygons

  const auto point_size = input->points.size();//subscribeされた点群のtopic?
  std::vector<bool> within(point_size, true);
#pragma omp parallel for
  for (std::size_t point_i = 0; point_i < point_size; ++point_i) {
    const auto point = input->points[point_i];
    bool _within = false;
    for (std::size_t area_i = 0; area_i < polygon_size; area_i++) {
      if (!area_check[area_i]) continue;

      if (boost::geometry::within(PointXY(point.x, point.y), area_polygons_[area_i])) {
        if(filter_type == 1){ _within = true;}
      }
    }

    within[point_i] = _within;//delete all領域に入っているか
  }

  for (std::size_t point_i = 0; point_i < point_size; ++point_i) {
    if (!within[point_i]) {
      const auto point = input->points[point_i];
      output->points.emplace_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
  }
}


bool MapAreaFilterComponent::filter_objects_by_area(PredictedObjects & out_objects)//各オブジェクトがオブジェクト削除領域に存在するなら消すという関数
{
  if (!objects_ptr_.has_value() || objects_ptr_.get() == nullptr) {
    return false;
  }//objects_ptr_はinput_objectsのpointer

  const PredictedObjects in_objects = *objects_ptr_.get();//input_objectのデータ
  out_objects.header = in_objects.header;
  for (const auto & object : in_objects.objects) {//各オブジェクトに対して
    const auto pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto object_label = object.classification[0].label;
    bool within = false;
    for (std::size_t area_i = 0, size = area_polygons_.size(); area_i < size; ++area_i) {//各領域に対して
      if ((boost::geometry::within(PointXY(pos.x, pos.y), area_polygons_[area_i]))) {//各領域にオブジェクトの重心が入っているなら
        if(object_label == area_labels[area_i] || area_labels[area_i] == (uint8_t)8){ within = true; }
      }
    }
    if (!within) {//各オブジェクトが領域に属しているか(within==trueか)
      out_objects.objects.emplace_back(object);//属していないなら消去しない(outputする)
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

  if (!csv_loaded_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Areas CSV Not yet loaded");
  }
  if (!do_filter_ || area_polygons_.empty()) {
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
  if (get_param(p, "area_distance_check", area_distance_check_)) {
    RCLCPP_DEBUG(
      this->get_logger(), "Setting new area check distance to: %.2lf.", area_distance_check_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace map_area_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_area_filter::MapAreaFilterComponent);
