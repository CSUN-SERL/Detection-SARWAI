#include "normalizer_handler.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace sarwai {
  NormalizerHandler::NormalizerHandler(ros::NodeHandle * nh, unsigned newwidth, unsigned newheight):
      nh_(nh),
      it_(*nh_),
      new_width_(newwidth),
      new_height_(newheight) {
    sub_ = it_.subscribe("/stereo/image_rect", 1000, &NormalizerHandler::NormalizePointcloud, this);
    publish_destination_ = "/sarwai_detection/normalized_pointcloud";
    pub_ = it_.advertise(publish_destination_, 1000);
  }

  void NormalizerHandler::NormalizePointcloud(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("Normalizer received image");
    //Copy message to new Image
    sensor_msgs::Image image_copy = (*msg);

    //Convert message to CvImage
    cv_bridge::CvImagePtr cv_image;
    cv_image = cv_bridge::toCvCopy(image_copy, image_copy.encoding.c_str());

    //Extract image matrix
    cv::Mat image_matrix = cv_image->image;

    //Resize image, store in "resized"
    cv::Mat resized;
    cv::Size new_size(this->new_width_, this->new_height_);
    cv::resize(image_matrix, resized, new_size, 0, 0, cv::INTER_LINEAR);

    //Convert back to sensor_msgs::Image
    sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(image_copy.header, image_copy.encoding, resized).toImageMsg();

    //Publish the new image
    image_copy = *output_image;
    while(!pub_.getNumSubscribers()){
      ros::Duration(0.1).sleep();
    }
    pub_.publish(image_copy);
    ROS_INFO("Normalizer published the image");
  }
}