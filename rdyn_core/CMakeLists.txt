#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)
get_project_name("package.xml" extracted_name extracted_version)
project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)
#############################
## END - PRELIMINARIES     ##
#############################


#############################
## OPTIONS                 ##
#############################
option(ENABLE_TESTING           "ENABLE TESTING" ON)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(BUILD_EXAMPLE            "COMPILE THE EXAMPLE" ON)
option(MAX_NUM_AXES             "Change from allocation in heap (MAX_NUM_AXES=-1) to allocation in stack (MAX_NUM_AXES>0)." -1)
option(USE_RAW_POINTERS         "Use raw pointer instead of shared pointer (code optimization)" FALSE)
#############################
## END - OPTIONS           ##
#############################


#############################
## DEPENDANCIES            ##
#############################
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(cnr_control_toolbox REQUIRED COMPONENT eigen_matrix_utils)

find_package(PkgConfig REQUIRED)
pkg_check_modules(urdfdom REQUIRED urdfdom IMPORTED_TARGET)

#cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)


#if(ENABLE_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  include(CTest)

#endif()


#############################
## END DEPENDANCIES        ##
#############################

#################
## Build       ##
#################

add_library(rdyn_core INTERFACE)
target_include_directories(rdyn_core
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> #if BUILD_INTERFACE, add ${CMAKE_SOURCE_DIR}/include (headers taken by source directory)
    $<INSTALL_INTERFACE:include> # if INSTALL_INTERFACE, add header from install path
  )
target_compile_definitions(rdyn_core
  INTERFACE
  MAX_NUM_AXES=-1
  $<$<BOOL:${USE_RAW_POINTERS}>:USE_RAW_POINTERS=1>
  )



target_link_libraries(rdyn_core
  INTERFACE
  Boost::filesystem
  Boost::system
  PkgConfig::urdfdom
  eigen_matrix_utils::eigen_matrix_utils
)

add_library(rdyn_core::rdyn_core ALIAS rdyn_core)

# Compiled version of the library as a shared library
add_library(rdyn_core_compiled SHARED
  src/rdyn_core.cpp # This cpp file includes your _impl.h
)

target_compile_options(rdyn_core_compiled PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

# Use the same include directories as the header-only library
target_include_directories(rdyn_core_compiled
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link to the same dependencies as the header-only library
target_link_libraries(rdyn_core_compiled
  PUBLIC
    rdyn_core::rdyn_core # Link to the INTERFACE target to inherit compile definitions and other properties
)

# Optionally, if needed, set the versioning properties for shared library
set_target_properties(rdyn_core_compiled PROPERTIES
  VERSION 1.0
  SOVERSION 1
)

add_library(rdyn_core::rdyn_core_compiled ALIAS rdyn_core_compiled)



#################
## END - Build ##
#################


###################
## Testing       ##
###################
#if(ENABLE_TESTING)
  message(INFO "ENABLE_TESTING is enabled")
  add_executable(rdyn_speed_test test/rdyn_speed_test.cpp)
  target_link_libraries(rdyn_speed_test
    PUBLIC
    rdyn_core::rdyn_core
    GTest::gtest_main
  )

  target_compile_options(rdyn_speed_test PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)
  target_compile_definitions(rdyn_speed_test
    PUBLIC PROJECT_SRC_DIRECTORY="${CMAKE_INSTALL_PREFIX}/share/${LIBRARY_NAME}/testing"
  )

  #target_compile_options(${LIBRARY_NAME}_test PUBLIC $<$<CONFIG:Debug>>)
  gtest_discover_tests(rdyn_speed_test)
  add_test(NAME rdyn_speed_test
    COMMAND rdyn_speed_test)


  add_executable(rdyn_speed_test_compiled test/rdyn_speed_test.cpp)
  target_link_libraries(rdyn_speed_test_compiled
    PUBLIC
    rdyn_core::rdyn_core_compiled
    GTest::gtest_main
  )

  target_compile_definitions(rdyn_speed_test_compiled
    PUBLIC PROJECT_SRC_DIRECTORY="${CMAKE_INSTALL_PREFIX}/share/${LIBRARY_NAME}/testing"
  )

  #target_compile_options(${LIBRARY_NAME}_test PUBLIC $<$<CONFIG:Debug>>)
  gtest_discover_tests(rdyn_speed_test_compiled)
  add_test(NAME rdyn_speed_test_compiled
    COMMAND rdyn_speed_test_compiled)


  add_executable(${PROJECT_NAME}_test test/test.cpp)
  target_link_libraries(${PROJECT_NAME}_test rdyn_core::rdyn_core
    GTest::gtest_main
    Threads::Threads)
  target_compile_definitions(${PROJECT_NAME}_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")
  target_compile_options(${PROJECT_NAME}_test PUBLIC $<$<CONFIG:Release>:-Ofast >)
  cnr_target_compile_options(${PROJECT_NAME}_test)
  target_compile_definitions(${PROJECT_NAME}_test
    PUBLIC PROJECT_SRC_DIRECTORY="${CMAKE_INSTALL_PREFIX}/share/${LIBRARY_NAME}/testing"
  )
  gtest_discover_tests(${PROJECT_NAME}_test)
  add_test(NAME ${PROJECT_NAME}_test
    COMMAND test_filters)

#endif()
###################
## END - Testing ##
###################


### #############
### ## Install ##
### #############


install(DIRECTORY include/rdyn_core
  DESTINATION include)

install(FILES test/ur10.urdf
  DESTINATION share/testing)

install(
  TARGETS rdyn_core rdyn_core_compiled rdyn_speed_test rdyn_speed_test_compiled
  EXPORT rdyn_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT rdyn_coreTargets
  DESTINATION "share/rdyn_core/cmake"
  NAMESPACE rdyn_core::
  FILE rdyn_coreTargets.cmake
)


include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/rdyn_coreConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)

install(FILES
  ${CMAKE_SOURCE_DIR}/cmake_config/rdyn_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/rdyn_coreConfigVersion.cmake
  DESTINATION "share/rdyn_core/cmake")


# make uninstall
add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)


### install(DIRECTORY include/${PROJECT_NAME}
###         DESTINATION ${CNR_INSTALL_INCLUDE_DIR}
###         FILES_MATCHING PATTERN "*.h"
###         PATTERN ".git" EXCLUDE
### )
###
### install(TARGETS ${PROJECT_NAME}
###         ARCHIVE DESTINATION ${CNR_INSTALL_LIB_DIR}
###         LIBRARY DESTINATION ${CNR_INSTALL_LIB_DIR}
###         RUNTIME DESTINATION ${CNR_INSTALL_BIN_DIR}
### )
###
### ###################
### ## END - Install ##
### ###################

##### ##################################
##### ## CMAKE CONFIG (IF NOT CATKIN) ##
##### ##################################
##### if(NOT USE_ROS1)
#####
#####   install(
#####     TARGETS ${PROJECT_NAME}
#####     EXPORT export_rdyn_core
#####     ARCHIVE DESTINATION lib
#####     LIBRARY DESTINATION lib
#####     RUNTIME DESTINATION bin
#####   )
#####
#####   # export targets
#####   set(export_targets ${export_targets};rdyn_core_utilities)
#####   set(export_targets ${export_targets};rdyn_core_server)
#####   export(EXPORT export_rdyn_core
#####     FILE "${PROJECT_BINARY_DIR}/export_rdyn_core.cmake")
#####
#####   # Create the rdyn_coreConfig.cmake
#####   set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
#####   configure_file(rdyn_coreConfig.cmake.in
#####     "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/rdyn_coreConfig.cmake" @ONLY)
#####
#####   # Create the rdyn_coreConfigVersion.cmake
#####   configure_file(rdyn_coreConfigVersion.cmake.in
#####     "${PROJECT_BINARY_DIR}/rdyn_coreConfigVersion.cmake" @ONLY)
#####
#####   install(FILES
#####     "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/rdyn_coreConfig.cmake"
#####     "${PROJECT_BINARY_DIR}/rdyn_coreConfigVersion.cmake"
#####     DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev
#####   )
#####
#####   install(EXPORT export_rdyn_core
#####     DESTINATION "share/${PROJECT_NAME}/cmake"
#####     FILE export_rdyn_core.cmake
#####     NAMESPACE rdyn_core::
#####     COMPONENT dev
#####   )
##### endif()
##### ##################################
##### ## CMAKE CONFIG (IF NOT CATKIN) ##
##### ##################################
#####

