cmake_minimum_required(VERSION 2.8.3)
project(pr2_head_manager_msgs)

find_package(catkin REQUIRED
  roscpp
  message_generation
  resource_management_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StateMachineStatePoint.msg
  StateMachineStatePitchYaw.msg
  StateMachineStatePrioritizedPitch.msg
  StateMachineStatePrioritizedYaw.msg
  Point.msg PitchYaw.msg
  PrioritizedPitch.msg
  PrioritizedYaw.msg
  Pitch.msg
  RawPitchYaw.msg
  Yaw.msg
  StateMachine.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StateMachineRegister.srv
  StateMachineExtract.srv
)

generate_messages(DEPENDENCIES resource_management_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs)
