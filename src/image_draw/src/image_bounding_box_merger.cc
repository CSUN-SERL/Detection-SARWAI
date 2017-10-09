#include "image_bounding_box_merger.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace sarwai {

  ImageBoundingBoxMerger::ImageBoundingBoxMerger() {
    //Subscribes to darknet_ros/detection_image
    this->nh_ = new ros::NodeHandle();
    this->image_frame_sub_ = this->nh_->subscribe(
      "darknet_ros/detection_image", 1, &ImageBoundingBoxMerger::ImageCallback, this); 
    //subscribes to darknet_ros/bounding_boxes    
    this->bounding_box_sub_ = this->nh_->subscribe(
      "darknet_ros/bounding_boxes", 1000, &ImageBoundingBoxMerger::ArrayReceived, this); 
    //subscribes to darknet_ros/found_object
    this->detection_flag_sub_ = this->nh_->subscribe(
      "darknet_ros/found_object", 1000, &ImageBoundingBoxMerger::ObjectDetected, this); 
    //Publishes to visual_detection topic
      this->visual_detection_pub_ = this->nh_->advertise<detection_msgs::ProcessedVisualDetection>(
        "visual_detection", 1000); 
  }
  
  ImageBoundingBoxMerger::~ImageBoundingBoxMerger() {
    //empty
  }
  
  void ImageBoundingBoxMerger::PublishMergedData(
    sensor_msgs::Image image, darknet_ros_msgs::BoundingBox box
  ) {
    detection_msgs::ProcessedVisualDetection outgoing_msg;
    //Set image info to custom message detection_msgs::ProcessedVisualDetection
    outgoing_msg.image = image; 
    //Set bouding box info to custom message detection_msgs::ProcessedVisualDetection
    outgoing_msg.bounding_box = box;    
    //Publishes to topic
    this->visual_detection_pub_.publish(outgoing_msg);  
  }
    //Recives images from topic
  void ImageBoundingBoxMerger::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {  
    //IMage is pushes into queue 
    this->video_image_frames_.push(*msg);   
    RunImageProcess();
  }

  void ImageBoundingBoxMerger::RunImageProcess() {
    //Process only if detection_flag_ queue and video_imag queue is not empty
    if (
        !this->detection_flag_.empty() &&       
        !this->video_image_frames_.empty()
    ) {
      //Check if the frame has detections in it.
      //A 1 indicates detections in image frame, 0 indicates no detections.
      if (this->detection_flag_.front() == 1) {     //1 means we have data to process
        if (!bounding_boxes_.empty()) { //Process if bounding_box queue is not empty
             //seting item in front of queue to bounding_box
          std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes = this->bounding_boxes_.front();  
          sensor_msgs::Image master_image = this->video_image_frames_.front();
          for (int i = 0; i < bounding_boxes.size(); i++) {
            DrawRectAndPublishImage(bounding_boxes[i], master_image);    
          }
            //Pops first bounding box information
          this->bounding_boxes_.pop();  
            //pops first frame of image
          this->video_image_frames_.pop();  
        }
      }
    }
  }
    //gets data from topic and pushes into detection_flag queue
  void ImageBoundingBoxMerger::ObjectDetected(const std_msgs::Int8& msg) { 
    this->detection_flag_.push(msg.data); //pushes data to queue
  }
    // gets bounding box data from the top and pushes into bounding_box queue
  void ImageBoundingBoxMerger::ArrayReceived(const darknet_ros_msgs::BoundingBoxes& msg) {  
    std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes = msg.boundingBoxes;
    for (int i = 0; i < bounding_boxes.size(); ++i) {
     //Processes frames only if they are labeled person
      if (bounding_boxes[i].Class != "person") {    
        bounding_boxes.erase(bounding_boxes.begin()+i);
      }
    }
    //Push bounding box to bounding_box queue
    this->bounding_boxes_.push(bounding_boxes);   
  }
    // Function draws box around the detected image
  void ImageBoundingBoxMerger::DrawRectAndPublishImage( 
    const darknet_ros_msgs::BoundingBox &box, const sensor_msgs::Image &image
  ) {
    // Create a value copy of the image
    sensor_msgs::Image image_copy = image;
    // Create an OpenCV image matrix from the ROS Image msg
    cv_bridge::CvImagePtr cv_image;
    cv_image = cv_bridge::toCvCopy(image_copy, sensor_msgs::image_encodings::BGR8);
    cv::Mat image_matrix = cv_image->image;
    
    // Draw the bounding box to the OpenCV matrix
    //CV draw function
    cv::Point top_left_corner = cv::Point(box.xmin, box.ymin);  
    //CV draw function
    cv::Point bottom_right_corner = cv::Point(box.xmax, box.ymax);  
    cv::rectangle(image_matrix, top_left_corner, bottom_right_corner, 2);
   
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_matrix).toImageMsg();
    image_copy = *image_msg;
    //reassigns header value as the transition to cv and back drops the header data.
    image_copy.header = image.header;
    PublishMergedData(image_copy, box);
  }
}
