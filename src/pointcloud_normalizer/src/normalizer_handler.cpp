#include "normalizer_handler.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace sarwai {
  NormalizerHandler::NormalizerHandler(ros::NodeHandle * nh, unsigned newwidth, unsigned newheight, std::string subscribe_destination, std::string publish_destination):
      nh_(nh),
      new_width_(newwidth),
      new_height_(newheight) {
    sub_ = nh_->subscribe(subscribe_destination, 1000, &NormalizerHandler::NormalizePointcloud, this);
    pub_ = nh_->advertise<detection_msgs::DetectionPointImage>(publish_destination, 1000);
  }

  void NormalizerHandler::NormalizePointcloud(const detection_msgs::DetectionPointImageConstPtr& msg) {
    ROS_INFO("Normalizer received image");
    //Copy message to new Image
    sensor_msgs::Image image_copy = msg->point_image;

    //Convert message to CvImage
    cv_bridge::CvImagePtr cv_image;
    cv_image = cv_bridge::toCvCopy(image_copy, image_copy.encoding.c_str());

    //Extract image matrix
    cv::Mat image_matrix = cv_image->image;

    //Resize image, store in "resized"
    cv::Mat resized;
    unsigned newwidth = this->new_width_ == 0 ? image_copy.width : new_width_;
    unsigned newheight = this->new_height_ == 0 ? image_copy.height : new_height_;

    cv::Size new_size(newwidth, newheight);
    cv::resize(image_matrix, resized, new_size, 0, 0, cv::INTER_LINEAR);

    //Convert back to sensor_msgs::Image
    sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(image_copy.header, image_copy.encoding, resized).toImageMsg();

    //Construct output message
    detection_msgs::DetectionPointImage out_msg;
    out_msg.detection = msg->detection;
    out_msg.point_image = *output_image;

    //Publish the new image
    while(!pub_.getNumSubscribers()){
      ros::Duration(0.1).sleep();
    }
    pub_.publish(out_msg);
    ROS_INFO("Normalizer published the image");
  }
}