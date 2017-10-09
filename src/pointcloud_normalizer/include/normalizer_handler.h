#ifndef SARWAI_NORMALIZER_HANDLER_H_
#define SARWAI_NORMALIZER_HANDLER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <string>

namespace sarwai {

  class NormalizerHandler {
  public:
    // NormalizerHandler(ros::NodeHandle * nh, unsigned originalwidth, unsigned originalheight, unsigned newwidth, unsigned newheight);
    NormalizerHandler(ros::NodeHandle * nh, unsigned newwidth, unsigned newheight);
  private:
    void NormalizePointcloud(const sensor_msgs::ImageConstPtr& msg);

    ros::NodeHandle * nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    unsigned new_width_;
    unsigned new_height_;
    std::string publish_destination_;
  };

}

#endif