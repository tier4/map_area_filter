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

#ifndef MAP_AREA_FILTER_CONTENTS_HPP_
#define MAP_AREA_FILTER_CONTENTS_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <pcl/common/impl/common.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/assign/std/vector.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/optional.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#include <random>

namespace map_area_filter
{
typedef boost::geometry::model::d2::point_xy<float> PointXY;
typedef boost::geometry::model::polygon<PointXY> Polygon2D;

using autoware_perception_msgs::msg::PredictedObjects;

/** \brief For parameter service callback */
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

class RemovalArea
{
public:
  lanelet::Id id_;
  std::unordered_set<std::string> target_labels_;
  lanelet::BasicPolygon2d polygon_;
  std::optional<double> min_removal_height_{std::nullopt};
  std::optional<double> max_removal_height_{std::nullopt};
  bool is_in_distance_{false};

  void update_is_in_distance(geometry_msgs::msg::Point pos, double distance);
};

class MapAreaFilterComponent : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

protected:
  void filter(const PointCloud2ConstPtr & input, PointCloud2 & output);

  bool do_filter_ = true;
  int filter_type;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  autoware::universe_utils::TransformListener transform_listener_{this};

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr area_markers_pub_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr filtered_objects_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr objects_cloud_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr lanelet_map_sub_;

  // private:
  std::shared_ptr<tf2_ros::Buffer> tf2_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::string map_frame_;
  std::string base_link_frame_;

  /** \brief The input PointCloud2 subscriber. */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher. */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  size_t max_queue_size_ = 3;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<RemovalArea> removal_areas_;

  visualization_msgs::msg::MarkerArray area_markers_msg_;

  nav_msgs::msg::Odometry kinematic_state_;
  autoware::route_handler::RouteHandler route_handler_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr objects_cloud_ptr_;
  boost::optional<PredictedObjects::ConstSharedPtr> objects_ptr_;

  rclcpp::TimerBase::SharedPtr timer_;

  double min_guaranteed_area_distance_;
  double marker_font_scale_;

  void filter_points_by_area(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
  bool filter_objects_by_area(PredictedObjects & out_objects);

  void timer_callback();
  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg);
  void lanelet_map_callback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg);
  void objects_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);
  void objects_callback(const PredictedObjects::ConstSharedPtr & cloud_msg);

  void color_func(double dis, std_msgs::msg::ColorRGBA & color);
  void create_area_marker_msg();

  static bool transform_pointcloud(
    const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
    const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output);

  void computePublish(const PointCloud2ConstPtr & input);

public:
  MapAreaFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace map_area_filter

#endif  // MAP_AREA_FILTER_CONTENTS_HPP_