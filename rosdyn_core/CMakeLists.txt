cmake_minimum_required(VERSION 2.8.3)
project(rosdyn_core)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  std_msgs roscpp urdf
)

find_package(Eigen3 REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES itia_dynamics_core 
  CATKIN_DEPENDS roscpp  std_msgs  urdf
  DEPENDS  
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(compute_inertia_matrix src/compute_inertia_matrix.cpp
)
add_dependencies(compute_inertia_matrix ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(compute_inertia_matrix
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
