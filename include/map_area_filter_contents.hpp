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

#pragma once
#include "basefilter.hpp"
#include "csv.hpp"

#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <pcl/common/impl/common.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/assign/std/vector.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/optional.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#include <random>

namespace map_area_filter
{
enum class AreaType {
  DELETE_ALL,     // Delete static and dynamic cloud
  DELETE_OBJECT,  // Delete detected object bbox
};

using autoware_perception_msgs::msg::DetectedObjects;

class MapAreaFilterComponent : public map_area_filter::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
    PointCloud2 & output);
  void subscribe() override;
  void unsubscribe() override;

  bool do_filter_, csv_loaded_ = true;
  int filter_type;
  bool csv_invalid =
    false;  // csv fileが開けない、pathが違うなどの問題が存在するか(正しくないならfilterしない

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  autoware::universe_utils::TransformListener transform_listener_{this};

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr area_markers_pub_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr filtered_objects_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr objects_cloud_sub_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr objects_sub_;

  // private:
  std::shared_ptr<tf2_ros::Buffer> tf2_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::string map_frame_;
  std::string base_link_frame_;
  typedef boost::geometry::model::d2::point_xy<float> PointXY;
  typedef boost::geometry::model::polygon<PointXY> Polygon2D;

  std::vector<AreaType> area_types_;
  std::vector<Polygon2D> area_polygons_;
  std::vector<uint8_t> area_labels;
  std::vector<PointXY> centroid_polygons_;
  std::deque<std::size_t> original_csv_order_;

  visualization_msgs::msg::MarkerArray area_markers_msg_;

  geometry_msgs::msg::PoseStamped current_pose_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr objects_cloud_ptr_;
  boost::optional<DetectedObjects::ConstSharedPtr> objects_ptr_;

  rclcpp::TimerBase::SharedPtr timer_;

  double area_distance_check_;
  double marker_font_scale_;

  /***
   * Returns true if valid polygons were found in the CSV
   * @param file_name CSV to parse and load polygons
   * @return true if valid polygons were found in the CSV
   */
  void csv_row_func(const csv::CSVRow & row, std::deque<csv::CSVRow> & rows, std::size_t row_i);
  void row_to_rowpoints(
    const csv::CSVRow & row, std::vector<PointXY> & row_points, AreaType & areatype,
    bool & correct_elem);

  bool load_areas_from_csv(const std::string & file_name);

  void filter_points_by_area(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
  bool filter_objects_by_area(DetectedObjects & out_objects);

  void timer_callback();
  void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg);
  void objects_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);
  void objects_callback(const DetectedObjects::ConstSharedPtr & cloud_msg);

  void color_func(double dis, std_msgs::msg::ColorRGBA & color);
  void create_area_marker_msg();

  static bool transform_pointcloud(
    const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
    const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output);

public:
  explicit MapAreaFilterComponent(const rclcpp::NodeOptions & options);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace map_area_filter
