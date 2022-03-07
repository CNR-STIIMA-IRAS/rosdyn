cmake_minimum_required(VERSION 2.8.3)
project(rosdyn_identification)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED COMPONENTS Core Dense)

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
  state_space_filters
  moveit_planning_helper
  roslint
  name_sorting
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
                  state_space_filters
                  moveit_planning_helper
                  roslint
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

set(ROSLINT_CPP_OPTS "--filter=-whitespace/line_length,-build/include_what_you_use,-runtime/references,-build/c++11")
roslint_cpp()

install(FILES rosdyn_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




