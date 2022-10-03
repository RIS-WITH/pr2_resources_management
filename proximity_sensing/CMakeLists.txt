cmake_minimum_required(VERSION 2.8.3)
project(proximity_sensing)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pr2_head_manager_msgs
  resource_management_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES proximity_sensing
  CATKIN_DEPENDS pr2_head_manager_msgs resource_management_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/proximity_sensing.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
