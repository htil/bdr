cmake_minimum_required(VERSION 2.8.3)
project(bebop_bdr_qlearn)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  )
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    LIBRARIES 
    CATKIN_DEPENDS roscpp
    DEPENDS
)

