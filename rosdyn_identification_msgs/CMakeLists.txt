cmake_minimum_required(VERSION 2.8.3)
project(rosdyn_identification_msgs)
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  std_msgs
  roscpp
  sensor_msgs
)

add_message_files(
  FILES
  TorqueFitting.msg
)
add_action_files(
  FILES
  MetoParEstim.action
  MetoTrjGen.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
)


catkin_package(
 CATKIN_DEPENDS actionlib actionlib_msgs std_msgs roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


