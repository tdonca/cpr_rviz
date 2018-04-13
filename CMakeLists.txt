cmake_minimum_required(VERSION 2.8.3)
project(cpr_rviz)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp rviz geometry_msgs std_msgs roslaunch)
find_package(Qt5Widgets)

## Find Boost (headers only)
find_package(Boost REQUIRED)

## Look for launch files
roslaunch_add_file_check(launch)

## Settings to load Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)



## Define catkin exports
catkin_package(
  #~ INCLUDE_DIRS include
  LIBRARIES cpr_rviz
  CATKIN_DEPENDS pluginlib roscpp rviz
  DEPENDS Boost 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

## pluginlib_tutorials library
add_library(cpr_rviz src/teleop_panel.cpp )
target_link_libraries(cpr_rviz ${catkin_LIBRARIES} Qt5::Widgets)

