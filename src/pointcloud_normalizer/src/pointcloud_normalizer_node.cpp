#include <ros/ros.h>
#include "normalizer_handler.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloudnormalizer");
  ros::NodeHandle nh;

  unsigned original_width;
  unsigned original_height;
  unsigned new_height;
  unsigned new_width;

  new_width = 800;
  new_height = 618;

  sarwai::NormalizerHandler handler(&nh, new_width, new_height);

  ros::spin();
}