#include <ros/ros.h>
#include "normalizer_handler.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloudnormalizer");
  ros::NodeHandle nh;

  unsigned new_height;
  unsigned new_width;

  new_width = 0;
  new_height = 0;

  sarwai::NormalizerHandler handler(&nh, new_width, new_height, "/sarwai_detection/detection_pointimage", "/sarwai_detection/detection_normalizedpointimage");

  ros::spin();
}