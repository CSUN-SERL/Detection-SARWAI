#include "detection_logger.h"
#include "logging_strategy_registry.h"
#include "local_image_logging_strategy.h"
#include <stdint.h>

namespace sarwai {
  
  DetectionLogger::DetectionLogger() {
    //empty
  }

  DetectionLogger::DetectionLogger(std::string image_topic_name, std::string audio_topic_name, ros::NodeHandle &nh) {  
    image_topic_name_ = image_topic_name;
    audio_topic_name_ = audio_topic_name;
    nh_ = &nh;
    image_sub_ = nh_->subscribe(image_topic_name_.c_str(), 1000, &DetectionLogger::ImageLogCallback, this);
    // audio_sub_ = nh_->subscribe(audio_topic_name_.c_str(), 1000, &DetectionLogger::AudioLogCallback, this);
    
    // temp, until we get a finalized way of choosing the strategy
    visual_logging_strategy_ = new LocalImageLoggingStrategy;
  }
  
  void DetectionLogger::InitLogEntryStruct(const detection_msgs::ProcessedVisualDetection::ConstPtr &msg,
    struct BoxMetadata &log_entry) {
    
    log_entry.box_height = msg->bounding_box.ymax - msg->bounding_box.ymin;
    log_entry.box_width = msg->bounding_box.xmax - msg->bounding_box.xmin;
    log_entry.left_x_coord = msg->bounding_box.xmin;
    log_entry.top_y_coord = msg->bounding_box.ymin;
    log_entry.timestamp = (int) msg->image.header.stamp.sec;
    log_entry.confidence_rating = msg->bounding_box.probability;
    log_entry.object_class = msg->bounding_box.Class;
  }

  void DetectionLogger::ImageLogCallback(
    const detection_msgs::ProcessedVisualDetection::ConstPtr& msg) {

    struct BoxMetadata log_entry;
    InitLogEntryStruct(msg, log_entry);
    sensor_msgs::Image image = msg->image;
    this->visual_logging_strategy_->Log(image, log_entry);
  }

  DetectionLogger::~DetectionLogger() {
    delete visual_logging_strategy_;
  }
};
