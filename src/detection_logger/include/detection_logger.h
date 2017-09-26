#ifndef SARWAI_DETECTION_LOGGER_DETECTION_LOGGER_H_
#define SARWAI_DETECTION_LOGGER_DETECTION_LOGGER_H_

#include "ros/ros.h"
#include "box_metadata.h"
#include "logging_strategy.h"
#include "detection_msgs/ProcessedVisualDetection.h"
#include <string>

namespace sarwai {
  class DetectionLogger {
  public:
    DetectionLogger();
    DetectionLogger(std::string topic_name);
    ~DetectionLogger();

  private:
    int msg_queue_limit_;
    std::string topic_name_;
    ros::NodeHandle* nh_;
    ros::Subscriber sub_;
  
    LoggingStrategy* logging_strategy_;

    void InitLogEntryStruct(const detection_msgs::ProcessedVisualDetection::ConstPtr &msg, struct BoxMetadata &log_entry_struct);
    void LogCallback(const detection_msgs::ProcessedVisualDetection::ConstPtr &msg);
  };

};

#endif
