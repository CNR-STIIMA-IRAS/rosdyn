cmake_minimum_required(VERSION 3.1)
project(rosdyn_core)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MAX_NUM_AXES "Change from allocation in heap (MAX_NUM_AXES=-1) to allocation in stack (MAX_NUM_AXES>0)." -1)
option(USE_RAW_POINTERS "Use raw pointer instead of shared pointer (code optimization)" FALSE)

if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
  add_compile_definitions(MAX_NUM_AXES=-1)
  if(USE_RAW_POINTERS)
    add_compile_definitions(USE_RAW_POINTERS=1)
  endif()
else()
  add_compile_options(-DMAX_NUM_AXES=${MAX_NUM_AXES})
  if(USE_RAW_POINTERS)
    add_compile_options(-DUSE_RAW_POINTERS=1)
  endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  std_msgs roscpp urdf eigen_matrix_utils
)

find_package(Eigen3 REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp  std_msgs  urdf eigen_matrix_utils
  DEPENDS EIGEN3
)

#############
## Targets ##
#############

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## example of usage
add_executable(rosdyn_speed_test test/rosdyn_speed_test.cpp)
add_dependencies(rosdyn_speed_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosdyn_speed_test
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS rosdyn_speed_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


