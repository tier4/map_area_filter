#include "map_area_filter_contents.hpp"
#include"rclcpp/rclcpp.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc,argv); ////ROS2通信を初期化
    rclcpp::NodeOptions node_options;

    rclcpp::spin(std::make_shared<map_area_filter::MapAreaFilterComponent>(node_options));
    
    return 0;
}