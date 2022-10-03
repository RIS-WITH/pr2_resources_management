cmake_minimum_required(VERSION 3.0.2)
project(mocap_filter_head_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  resource_management_msgs
  pr2_head_manager_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  geometry_msgs
  roscpp
)

find_package(optitrack_ros REQUIRED)

catkin_package(
)

include_directories(
  include/mocap_filter_head_manager
  ${catkin_INCLUDE_DIRS}
  ${optitrack_ros_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/mocap_filter.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME mocap_filter PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)