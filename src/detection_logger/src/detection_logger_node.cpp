#include "ros/ros.h"
#include "box_metadata.h"
#include "logging_strategy.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "loggerNodeDefaultName");
  ros::NodeHandle nh;

  ROS_INFO("Hello, world!");
  ros::spin();

  return 0;
}
