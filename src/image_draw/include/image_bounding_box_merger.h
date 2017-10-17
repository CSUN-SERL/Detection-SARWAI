#ifndef SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_
#define SARWAI_IMAGE_DRAW_IMAGE_BOUNDING_BOX_MERGER_H_

#include <vector>
#include <queue>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Image.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "detection_msgs/DetectionPointCloud.h"
#include "detection_msgs/PointCloudImage.h"

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
    //Queue hold series of 1s and 0s
    std::queue<int> detection_flag_;  
    //Queue hold video frames of type sensor_msgs::Image
    std::queue<sensor_msgs::Image> video_image_frames_; 
    //queue of bounding box information
    std::queue<std::vector<darknet_ros_msgs::BoundingBox>> bounding_boxes_;  
    //Publishes data
    void PublishMergedData(sensor_msgs::Image, darknet_ros_msgs::BoundingBox, const sensor_msgs::PointCloud2& cloud); 
    void RunImageProcess(const sensor_msgs::PointCloud2& cloud);
    void ImageAndBoundingBoxToPublishQueue(darknet_ros_msgs::BoundingBox,
      sensor_msgs::Image);
    void ImageCallback(const detection_msgs::PointCloudImageConstPtr& msg);
    void ArrayReceived(const darknet_ros_msgs::BoundingBoxes& msg);
    void ObjectDetected(const std_msgs::Int8& msg);
    void DrawRectAndPublishImage(const darknet_ros_msgs::BoundingBox &box, const sensor_msgs::Image &, const sensor_msgs::PointCloud2& cloud);
  };
}

#endif
