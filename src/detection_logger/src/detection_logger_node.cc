#include "ros/ros.h"
#include "box_metadata.h"
#include "detection_logger.h"
#include <string>


int main(int argc, char **argv) {
  ros::init(argc, argv, "visual_logger");
  std::string str = "placeholder_topic_name";
  sarwai::DetectionLogger logger(str);
  sarwai::DetectionLogger logger_test();
  ros::spin();
}