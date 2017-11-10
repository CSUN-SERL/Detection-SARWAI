#include "ros/ros.h"
#include "box_metadata.h"
#include "detection_logger.h"
#include <string>


int main(int argc, char **argv) {
  ros::init(argc, argv, "visual_logger");
  ros::NodeHandle nh;
  std::string visual_topic_name = "/sarwai_detection/detection_processeddetection";
  std::string audio_topic_name = "audio_detection";
  sarwai::DetectionLogger logger(visual_topic_name, audio_topic_name, nh);
  //sarwai::DetectionLogger logger_test();
  ros::spin();
}
