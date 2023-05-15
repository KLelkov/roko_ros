cmake_minimum_required(VERSION 2.8.3)
project(rdk_msgs)

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   message_generation
)

add_message_files(
  FILES
  trajectory_point.msg
  locomotion.msg
  navigation.msg
  navigation_full.msg
  control.msg
  target.msg
  smart_target.msg
  motion_command.msg
  waypoint_status.msg
  led_state.msg
  weather.msg
  motors.msg
  avoiding_data.msg
  us_data.msg
)

add_service_files(
  FILES
  rokoRequest.srv
  waypoint.srv
  landingRequest.srv
  genericRequest.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS
  	message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
