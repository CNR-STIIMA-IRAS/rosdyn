#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 2.8.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

get_project_name("package.xml" extracted_name extracted_version)

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0.2)
  project(${extracted_name})
else()
  project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)
endif()

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
option(BUILD_EXAMPLE            "COMPILE THE EXAMPLE" OFF)
option(MAX_NUM_AXES             "Change from allocation in heap (MAX_NUM_AXES=-1) to allocation in stack (MAX_NUM_AXES>0)." -1)
option(USE_RAW_POINTERS         "Use raw pointer instead of shared pointer (code optimization)" FALSE)

if(USE_ROS1)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
endif()

if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
  add_compile_definitions(MAX_NUM_AXES=-1)
  if(USE_RAW_POINTERS)
    add_compile_definitions(USE_RAW_POINTERS=1)
  endif()
  if(USE_ROS1)
    add_compile_definitions(USE_ROS1=1)
  endif()
else()
  add_compile_options(-DMAX_NUM_AXES=${MAX_NUM_AXES})
  if(USE_RAW_POINTERS)
    add_compile_options(-DUSE_RAW_POINTERS=1)
  endif()
  if(USE_ROS1)
    add_compile_options(-DUSE_ROS1=1)
  endif()
endif()
#############################
## END - OPTIONS           ##
#############################


#############################
## DEPENDANCIES            ##
#############################
find_package(Eigen3 REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS system filesystem)

if(USE_ROS1)
  find_package(catkin REQUIRED COMPONENTS urdf eigen_matrix_utils kinematics_filters)

  catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS urdf eigen_matrix_utils kinematics_filters)
else()
  find_package(eigen_matrix_utils REQUIRED)
  find_package(kinematics_filters REQUIRED)
  find_package(urdf REQUIRED)
endif()

cnr_install_directories(USE_ROS1
  CNR_INSTALL_INCLUDE_DIR CNR_INSTALL_LIB_DIR CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)
#############################
## END DEPENDANCIES        ##
#############################

#################
## Build       ##
#################
list(APPEND DEPENDENCIES_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
list(APPEND DEPENDENCIES_LIBRARIES ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
if(USE_ROS1)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
  list(APPEND DEPENDENCIES_LIBRARIES ${catkin_LIBRARIES})
else()
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${eigen_matrix_utils_INCLUDE_DIRS} ${kinematics_filters_INCLUDE_DIRS} ${urdf_INCLUDE_DIRS})
  list(APPEND DEPENDENCIES_LIBRARIES ${eigen_matrix_utils_LIBRARIES} ${kinematics_filters_LIBRARIES}  ${urdf_LIBRARIES})
endif()

list(APPEND BUILD_INTERFACE_INCLUDE_DIRS ${DEPENDENCIES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS include ${DEPENDENCIES_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME} src/rdyn_core.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC 
  "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")
target_link_libraries(${PROJECT_NAME} "${DEPENDENCIES_LIBRARIES}")

if(BUILD_EXAMPLE)
## example of usage
  add_executable(rdyn_speed_test test/rdyn_speed_test.cpp)
  target_link_libraries(rdyn_speed_test ${PROJECT_NAME})
  if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
    add_compile_definitions(PROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
  else()
    add_compile_options(-DPROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
endif()
#################
## END - Build ##
#################

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CNR_INSTALL_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CNR_INSTALL_LIB_DIR}
        LIBRARY DESTINATION ${CNR_INSTALL_LIB_DIR}
        RUNTIME DESTINATION ${CNR_INSTALL_BIN_DIR}
)

if(USE_ROS1)
  install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
endif()
###################
## END - Install ##
###################

##################################
## CMAKE CONFIG (IF NOT CATKIN) ##
##################################
if(NOT USE_ROS1)
  
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_rdyn_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # export targets
  set(export_targets ${export_targets};rdyn_core_utilities)
  set(export_targets ${export_targets};rdyn_core_server)
  export(EXPORT export_rdyn_core
    FILE "${PROJECT_BINARY_DIR}/export_rdyn_core.cmake")

  # Create the rdyn_coreConfig.cmake
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  configure_file(rdyn_coreConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/rdyn_coreConfig.cmake" @ONLY)

  # Create the rdyn_coreConfigVersion.cmake
  configure_file(rdyn_coreConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/rdyn_coreConfigVersion.cmake" @ONLY)
  
  install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/rdyn_coreConfig.cmake"
    "${PROJECT_BINARY_DIR}/rdyn_coreConfigVersion.cmake"
    DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev
  )

  install(EXPORT export_rdyn_core
    DESTINATION "share/${PROJECT_NAME}/cmake"
    FILE export_rdyn_core.cmake
    COMPONENT dev
  )
endif()
##################################
## CMAKE CONFIG (IF NOT CATKIN) ##
##################################

###################
## Testing       ##
###################
if(ENABLE_TESTING)
      
  if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
      add_compile_definitions(PROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
    else()
      add_compile_options(-DPROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
  endif()

  if(USE_ROS1)
    add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${rostest_LIBRARIES} )
    target_compile_options(${PROJECT_NAME}_test PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)
    

    add_rostest_gtest(${PROJECT_NAME}_test_vectorization test/test_vectorization.launch test/test.cpp)
    target_link_libraries(${PROJECT_NAME}_test_vectorization ${PROJECT_NAME} ${rostest_LIBRARIES} )
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
  else()
    include_directories(${GTEST_INCLUDE_DIRS})
    add_executable(${PROJECT_NAME}_test test/test.cpp)
    target_include_directories(${PROJECT_NAME}_test PUBLIC
                            "$<BUILD_INTERFACE:${GTEST_INCLUDE_DIRS}>"
                            "$<INSTALL_INTERFACE:${GTEST_INCLUDE_DIRS}>")
    if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
      target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} gtest Threads::Threads)
    else()
      target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} gtest)
      if(THREADS_HAVE_PTHREAD_ARG)
        target_compile_options(${PROJECT_NAME}_test PUBLIC "-pthread")
      endif()
      if(CMAKE_THREAD_LIBS_INIT)
        target_link_libraries(${PROJECT_NAME}_test "${CMAKE_THREAD_LIBS_INIT}")
      endif()
    endif()
    target_compile_definitions(${PROJECT_NAME}_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")
    cnr_target_compile_options(${PROJECT_NAME}_test)

    gtest_add_tests(${PROJECT_NAME}_test "${CMAKE_SOURCE_DIR}/test/config/" AUTO)
  endif()
endif()
###################
## END - Testing ##
###################
