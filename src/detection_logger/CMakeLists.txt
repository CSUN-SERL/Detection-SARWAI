cmake_minimum_required(VERSION 2.8.3)
project(detection_logger)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  detection_msgs
  cv_bridge
image_transport
)

catkin_package(
  CATKIN_DEPENDS roscpp detection_msgs cv_bridge image_transport
)

include_directories(
  include
  include/audio
  include/visual
  ${catkin_INCLUDE_DIRS}
)

set(
  detection_logger_SOURCES
  src/detection_logger.cc
  src/logging_strategy_registry.cc
  src/visual/local_image_logging_strategy.cc

)

## Create executables and linker dependencies
add_executable(visual_logger src/detection_logger_node.cc ${detection_logger_SOURCES})
target_link_libraries(visual_logger ${catkin_LIBRARIES})
add_dependencies(visual_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
