cmake_minimum_required(VERSION 2.8.3)
project(rosdyn_identification)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O4")
# set(CMAKE_BUILD_TYPE Debug)

#add_compile_options(-DUBUNTU1404)


find_package(Eigen3 REQUIRED COMPONENTS Core Dense)


# add_definitions ( -DROSVERSIONNAME=$ENV{ROS_DISTRO} )

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  control_msgs
  geometry_msgs
  rosdyn_core
  rosdyn_identification_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  nodelet
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  eigen_matrix_utils
  eigen_state_space_systems
  moveit_planning_helper
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS  actionlib
                  control_msgs
                  geometry_msgs
                  geometric_shapes
                  rosdyn_core
                  rosdyn_identification_msgs
                  moveit_core
                  moveit_ros_planning
                  moveit_ros_planning_interface
                  nodelet
                  pluginlib
                  roscpp
                  sensor_msgs
                  std_msgs
                  trajectory_msgs
                  eigen_matrix_utils
                  eigen_state_space_systems
                  moveit_planning_helper
  DEPENDS
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
            src/${PROJECT_NAME}/rosdyn_par_estim.cpp
            src/${PROJECT_NAME}/rosdyn_par_estim_interface.cpp
            src/${PROJECT_NAME}/rosdyn_gen_interface.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)


