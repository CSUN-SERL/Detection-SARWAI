#ifndef SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_
#define SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_

#include <vector>
#include <queue>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Image.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "detection_msgs/ProcessedVisualDetection.h"

namespace sarwai {

  class ImageBoundingBoxMerger {
  public:

    ImageBoundingBoxMerger();
    ~ImageBoundingBoxMerger();
  
  private:

    ros::NodeHandle* nh_;

    ros::Subscriber image_frame_sub_;
    ros::Subscriber bounding_box_sub_;
    ros::Subscriber detection_flag_sub_;

    ros::Publisher visual_detection_pub_;

    std::queue<int> detection_flag_;
    std::queue<sensor_msgs::Image> video_image_frames_;
    std::queue<darknet_ros_msgs::BoundingBox> bounding_boxes_;
    std::queue<detection_msgs::ProcessedVisualDetection> merged_detections_;

    int count = 0;

    void PublishMergedData(sensor_msgs::Image, darknet_ros_msgs::BoundingBox);
    void RunImageProcess();
    void ImageAndBoundingBoxToPublishQueue(darknet_ros_msgs::BoundingBox,
      sensor_msgs::Image);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void ArrayReceived(const darknet_ros_msgs::BoundingBoxes& msg);
    void ObjectDetected(const std_msgs::Int8& msg);
    void DrawRect(const darknet_ros_msgs::BoundingBox &box, sensor_msgs::Image &);
  };
}

#endif