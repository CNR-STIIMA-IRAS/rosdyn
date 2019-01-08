cmake_minimum_required(VERSION 2.8.3)
project(rosdyn_gui)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rosdyn_identification_msgs
  roscpp
  rviz
  std_msgs
  std_srvs 
  moveit_planning_helper
)
catkin_package(
  CATKIN_DEPENDS rosdyn_identification_msgs roscpp std_msgs std_srvs moveit_planning_helper
)

link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)


include_directories(
src
${catkin_INCLUDE_DIRS}
${Qt5Widgets_INCLUDE_DIRS}
${Qt5Core_INCLUDE_DIRS})
set(SRC_FILES
  src/setup_popup.cpp
  src/rosdyn_gui.cpp
  src/dyn_components.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})



