cmake_minimum_required(VERSION 3.0.2)
project(robot_cleaner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  actionlib
  move_base_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)



