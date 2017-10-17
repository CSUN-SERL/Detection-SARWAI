#ifndef SARWAI_NORMALIZER_HANDLER_H_
#define SARWAI_NORMALIZER_HANDLER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <detection_msgs/DetectionPointImage.h>
#include <string>

namespace sarwai {

  class NormalizerHandler {
  public:
    NormalizerHandler(ros::NodeHandle * nh, unsigned newwidth, unsigned newheight, std::string subtopic, std::string pubtopic);
  private:
    void NormalizePointcloud(const detection_msgs::DetectionPointImageConstPtr& msg);

    ros::NodeHandle * nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    unsigned new_width_;
    unsigned new_height_;
    std::string publish_destination_;
  };

}

#endif