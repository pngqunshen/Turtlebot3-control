cmake_minimum_required(VERSION 2.8.3)
project(ee4308_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    geometry_msgs 
    message_generation 
    nav_msgs 
    roscpp 
    rospy 
    sensor_msgs 
    std_msgs 
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
