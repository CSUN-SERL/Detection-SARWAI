cmake_minimum_required(VERSION 2.8.3)
project(tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  darknet_ros_msgs
  rospy
  detection_msgs
  OpenCV REQUIRED
)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs darknet_ros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/image_tracker.cpp
  src/detection_tracker.cc
  src/face_classifier_manager.cc
  src/face_identifier_model.cc
  src/detection_frame_id.cc
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  "/usr/local/lib/libopencv_tracking.so"
  ${catkin_LIBRARIES}
)
