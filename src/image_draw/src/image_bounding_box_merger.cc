#include "image_bounding_box_merger.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace sarwai {

  ImageBoundingBoxMerger::ImageBoundingBoxMerger() {
    this->nh_ = new ros::NodeHandle();
    this->image_frame_sub_ = this->nh_->subscribe(
      "darknet_ros/detection_image", 1, &ImageBoundingBoxMerger::ImageCallback, this);
    this->bounding_box_sub_ = this->nh_->subscribe(
      "darknet_ros/bounding_boxes", 1000, &ImageBoundingBoxMerger::ArrayReceived, this);
    this->detection_flag_sub_ = this->nh_->subscribe(
      "darknet_ros/found_object", 1000, &ImageBoundingBoxMerger::ObjectDetected, this);

      this->visual_detection_pub_ = this->nh_->advertise<detection_msgs::ProcessedVisualDetection>(
        "visual_detection", 1000);
        ROS_INFO("constructor\n");
  }
  
  ImageBoundingBoxMerger::~ImageBoundingBoxMerger() {
    //empty
  }
  
  void ImageBoundingBoxMerger::PublishMergedData(
    sensor_msgs::Image image, darknet_ros_msgs::BoundingBox box
  ) {
    detection_msgs::ProcessedVisualDetection outgoing_msg;
    outgoing_msg.image = image;
    outgoing_msg.bounding_box = box;
    this->visual_detection_pub_.publish(outgoing_msg);
  }

  void ImageBoundingBoxMerger::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    this->video_image_frames_.push(*msg);
    RunImageProcess();
  }

  void ImageBoundingBoxMerger::RunImageProcess() {
    if (
        !this->detection_flag_.empty() &&
        !this->video_image_frames_.empty()
    ) {
      //Check if the frame has detections in it.
      //A 1 indicates detections in image frame, 0 indicates no detections.
      if (this->detection_flag_.front() == 1) {
        if (!bounding_boxes_.empty()) {
          ROS_INFO("hi\n");
          darknet_ros_msgs::BoundingBox bounding_box = this->bounding_boxes_.front();
          sensor_msgs::Image image = this->video_image_frames_.front();
          this->DrawRect(bounding_box, image);

          PublishMergedData(image, bounding_box);

          this->bounding_boxes_.pop();
          this->video_image_frames_.pop();
        }
      }
    }
  }

  void ImageBoundingBoxMerger::ObjectDetected(const std_msgs::Int8& msg) {
    this->detection_flag_.push(msg.data);
  }

  void ImageBoundingBoxMerger::ArrayReceived(const darknet_ros_msgs::BoundingBoxes& msg) {
    for (int i = 0; i < msg.boundingBoxes.size(); ++i) {
      if (msg.boundingBoxes[i].Class == "person") {
        this->bounding_boxes_.push(msg.boundingBoxes[i]);
      }
    }
  }

  void ImageBoundingBoxMerger::DrawRect(
    const darknet_ros_msgs::BoundingBox &box, sensor_msgs::Image &image
  ) {
    // Create an OpenCV image matrix from the ROS Image msg
    cv_bridge::CvImagePtr cv_image;
    cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    cv::Mat image_matrix = cv_image->image;
    // Draw the bounding box to the OpenCV matrix
    cv::Point top_left_corner = cv::Point(box.xmin, box.ymin);
    cv::Point bottom_right_corner = cv::Point(box.xmax, box.ymax);
    cv::rectangle(image_matrix, top_left_corner, bottom_right_corner, 2);
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_matrix).toImageMsg();
    image = *image_msg;
    // (*image) = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_matrix);
  }
}
