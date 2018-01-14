cmake_minimum_required(VERSION 2.8.3)
project(audio_trigger_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS nav_msgs
)
