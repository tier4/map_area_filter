cmake_minimum_required(VERSION 3.14)
project(map_area_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenMP)

ament_auto_add_library(map_area_filter SHARED
  src/map_area_filter_contents.cpp
)
ament_target_dependencies(map_area_filter)

if(OPENMP_FOUND)
  set_target_properties(map_area_filter PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "map_area_filter::MapAreaFilterComponent"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
