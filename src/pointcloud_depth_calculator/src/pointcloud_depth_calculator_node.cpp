#include <ros/ros.h>

#include "depth_calculator.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "depthcalculator");
  ros::NodeHandle nh;

  sarwai::DepthCalculator dc(&nh, "/visual_detection", "/sarwai_detection/detection_processeddetection");

  ros::spin();

  return 0;
}