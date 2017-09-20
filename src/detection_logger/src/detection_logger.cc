#include "detection_logger.h"
#include "logging_strategy_registry.h"
#include <stdint.h>

namespace sarwai {
  
  DetectionLogger::DetectionLogger() {
    //empty
  }

  DetectionLogger::DetectionLogger(std::string topic_name) {  
    topic_name_ = topic_name;
    nh_ = new ros::NodeHandle();
    sub_ = nh_->subscribe(topic_name_.c_str(), 1000, &DetectionLogger::LogCallback, this);
  }
  
  void DetectionLogger::InitLogEntryStruct(const detection_msgs::ImageBoundingBox::ConstPtr &msg,
    struct BoxMetadata &log_entry) {
    
    log_entry.box_height = msg->box_height;
    log_entry.box_width = msg->box_width;
    log_entry.left_x_coord = msg->box_x_coord;
    log_entry.top_y_coord = msg->box_y_coord;
    log_entry.timestamp = (int) msg->header.stamp.sec;
    log_entry.confidence_rating = msg->confidence;
    log_entry.object_class = msg->detected_class;
  }

  void DetectionLogger::LogCallback(
    const detection_msgs::ImageBoundingBox::ConstPtr& msg) {

    struct BoxMetadata log_entry;
    InitLogEntryStruct(msg, log_entry);
    this->logging_strategy_->Log(msg->image, log_entry);
  }

  DetectionLogger::~DetectionLogger() {
    delete nh_;
  }
};
