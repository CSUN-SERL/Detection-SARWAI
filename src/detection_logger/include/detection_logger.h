#ifndef SARWAI_DETECTION_LOGGER_DETECTION_LOGGER_H_
#define SARWAI_DETECTION_LOGGER_DETECTION_LOGGER_H_

#include "ros/ros.h"
#include "box_metadata.h"
#include "image_logging_strategy.h"
#include "detection_msgs/ProcessedVisualDetection.h"
#include <string>

namespace sarwai {
  class DetectionLogger {
  public:
    DetectionLogger();
    DetectionLogger(std::string image_topic_name, std::string audio_topic_name, ros::NodeHandle &nh);
    ~DetectionLogger();

  private:
    int msg_queue_limit_; // currently not being used, do we need it?
    std::string image_topic_name_;
    std::string audio_topic_name_;
    ros::NodeHandle* nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber audio_sub_;
  
    ImageLoggingStrategy* visual_logging_strategy_;
    // AudioLoggingStrategy* audio_logging_strategy;

    void InitLogEntryStruct(const detection_msgs::ProcessedVisualDetection::ConstPtr &msg, struct BoxMetadata &log_entry_struct);
    void ImageLogCallback(const detection_msgs::ProcessedVisualDetection::ConstPtr &msg);
    // void AudioLogCallback(const detection_msgs::ProcessedAudioDetection::ConstPtr &msg);
  };

};

#endif
