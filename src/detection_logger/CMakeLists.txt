cmake_minimum_required(VERSION 2.8.3)
project(detection_logger)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  detection_msgs
)

## Declare a catkin package
catkin_package()

## Tell linker where to include files from
include_directories(include ${catkin_INCLUDE_DIRS})

## Prepare the list of local source files that must be included in compilation
set(${PROJECT_NAME}_SOURCES
  src/detection_logger.cc
  src/local_box_data_saver.cc
  src/logging_strategy.cc
  )

## Create executables and linker dependencies
add_executable(visual_logger src/detection_logger_node.cc ${${PROJECT_NAME}_SOURCES})
target_link_libraries(visual_logger ${catkin_LIBRARIES})
add_dependencies(visual_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
