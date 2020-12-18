cmake_minimum_required(VERSION 3.0.2)
project(rosdyn_utilities)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  eigen_state_space_systems
  eigen_matrix_utils
  roscpp
  rosdyn_core
)

find_package(Eigen3 REQUIRED)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosdyn_utilities
  CATKIN_DEPENDS eigen_state_space_systems roscpp rosdyn_core 
  DEPENDS Eigen3::Eigen
)

###########
## Build ##
###########

include_directories(include  ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/chain_interface.cpp)
if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0") 
	target_precompile_headers(${PROJECT_NAME} PUBLIC  <sstream>
                            <type_traits>
                            <memory>
													  <string>
													  <vector>
													  <cmath>
													  <Eigen/Core>
													  <urdf_model/model.h>
													  <urdf_parser/urdf_parser.h> )
endif()
add_executable(${PROJECT_NAME}_example src/rosdyn_utilities_node_test.cpp)
set_target_properties(${PROJECT_NAME}_example PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen )

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${rostest_LIBRARIES} )

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()

