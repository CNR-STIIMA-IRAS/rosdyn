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
option(ENABLE_TESTING           "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(BUILD_EXAMPLE            "COMPILE THE EXAMPLE" ON)
option(MAX_NUM_AXES             "Change from allocation in heap (MAX_NUM_AXES=-1) to allocation in stack (MAX_NUM_AXES>0)." -1)
option(USE_RAW_POINTERS         "Use raw pointer instead of shared pointer (code optimization)" FALSE)
option(PYBIND                   "Create Python bindings" ON)
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

cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)
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


if(BUILD_EXAMPLE)
## example of usage
  add_executable(rdyn_speed_test test/rdyn_speed_test.cpp)
  target_link_libraries(rdyn_speed_test
    PUBLIC
    rdyn_core::rdyn_core)
  target_compile_definitions(rdyn_speed_test PUBLIC PROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
  cnr_target_compile_options(rdyn_speed_test)
endif()
#################
## END - Build ##
#################

##########################
## ## - Python Bindings ##
##########################

if(PYBIND)
  find_package(Python COMPONENTS REQUIRED
    Interpreter
    Development)

  find_package(pybind11 CONFIG REQUIRED)

  set(PYTHON_BINDINGS_PATH "${CMAKE_SOURCE_DIR}/python/bindings.cpp")
  pybind11_add_module(rdyn_py ${PYTHON_BINDINGS_PATH})

  target_include_directories(rdyn_py PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> #if BUILD_INTERFACE, add ${CMAKE_SOURCE_DIR}/include (headers taken by source directory)
    $<INSTALL_INTERFACE:include> # if INSTALL_INTERFACE, add header from install path
  )

  target_link_libraries(rdyn_py PRIVATE
    rdyn_core::rdyn_core
  )

  install(TARGETS rdyn_py
    DESTINATION lib/python3/site-packages)

  ## ## Colcon-specific source file
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)

  if (${Python_VERSION} VERSION_LESS "3.0.0")
    message(WARNING "Python 2 not supported!")
  else()
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv "prepend-non-duplicate;PYTHONPATH;lib/python3/site-packages")
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
  endif()
  ## End Colcon-specific

endif()

###########################
## END - Python Bindings ##
###########################

### #############
### ## Install ##
### #############


install(DIRECTORY include/rdyn_core
  DESTINATION include)

install(
  TARGETS rdyn_core
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

###################
## Testing       ##
###################
if(ENABLE_TESTING)

  find_package(GTest REQUIRED)
  include(GoogleTest)
  include(CTest)
    add_executable(${PROJECT_NAME}_test test/test.cpp)
    target_link_libraries(${PROJECT_NAME}_test rdyn_core::rdyn_core  GTest::gtest_main Threads::Threads)
    target_compile_definitions(${PROJECT_NAME}_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")
    target_compile_options(${PROJECT_NAME}_test PUBLIC $<$<CONFIG:Release>:-Ofast >)
    cnr_target_compile_options(${PROJECT_NAME}_test)
    target_compile_definitions(${PROJECT_NAME}_test PRIVATE PROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")

    gtest_discover_tests(${PROJECT_NAME}_test)
    add_test(NAME ${PROJECT_NAME}_test
      COMMAND test_filters)

endif()
###################
## END - Testing ##
###################
