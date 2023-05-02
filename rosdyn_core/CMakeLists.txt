#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

get_project_name("package.xml" extracted_name extracted_version)

project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)

cnr_set_flags()
#############################
## END - PRELIMINARIES     ##
#############################

#############################
## OPTIONS                 ##
#############################
option(USE_ROS1                 "ROS SUPPORT" ON)
option(ENABLE_TESTING           "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(COMPILE_EXAMPLE          "COMPILE THE EXAMPLE" OFF)
option(MAX_NUM_AXES             "Change from allocation in heap (MAX_NUM_AXES=-1) to allocation in stack (MAX_NUM_AXES>0)." -1)
option(USE_RAW_POINTERS          "Use raw pointer instead of shared pointer (code optimization)" FALSE)


if(USE_ROS1)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
endif()
#############################
## END - OPTIONS           ##
#############################

if(USE_ROS1)

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


  find_package(catkin REQUIRED COMPONENTS rdyn_core)
  find_package(Eigen3 REQUIRED COMPONENTS)
  find_package(Boost REQUIRED COMPONENTS system)

  ## Add gtest based cpp test target and link libraries
  if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS rdyn_core
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

  #############
  ## Testing ##
  #############

  ## Add gtest based cpp test target and link libraries
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_options(${PROJECT_NAME}_test PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

    add_rostest_gtest(${PROJECT_NAME}_test_vectorization test/test_vectorization.launch test/test.cpp)
    target_link_libraries(${PROJECT_NAME}_test_vectorization ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_options(${PROJECT_NAME}_test_vectorization PUBLIC
        $<$<CONFIG:Release>:-Ofast -shared -ffunction-sections -fdata-sections -funroll-loops -ffast-math -march=native -m64 -DEIGEN_NO_MALLOC>)
    if(${CMAKE_VERSION} VERSION_GREATER "3.16.0")
      target_link_options(${PROJECT_NAME}_test_vectorization PUBLIC
                                            $<$<CONFIG:Release>:-Wl,--gc-sections>)
    endif()
    if(ENABLE_COVERAGE_TESTING)
      set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
      add_code_coverage(
        NAME ${PROJECT_NAME}_coverage_report
        DEPENDENCIES ${PROJECT_NAME}_test
      )
    endif()
  endif()

endif(USE_ROS1)