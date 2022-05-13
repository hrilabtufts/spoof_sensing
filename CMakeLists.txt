cmake_minimum_required(VERSION 3.0.2)
project(spoof_sensing)

find_package(catkin REQUIRED COMPONENTS nav_msgs rospy sensor_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
