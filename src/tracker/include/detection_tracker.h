#ifndef SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_
#define SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_

#include <string>
#include <vector>
#include <memory>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "sensor_msgs/Image.h"
#include "detection_msgs/ProcessedVisualDetection.h"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/core/ocl.hpp>

namespace sarwai {

  enum TrackingAlgorithm {BOOSTING, MIL, KCF, TLD, MEDIANFLOW, GOTURN};
  
  class VisualDetectionTracker {

  public:
    void TrackFrame(const cv::Mat &image_matrix, std::vector<cv::Rect2d>);
    void AddTrackers(const cv::Mat &image_matrix, std::vector<cv::Rect2d>);
    VisualDetectionTracker();
    ~VisualDetectionTracker();
    
  private:
    ros::NodeHandle* nh_;
    TrackingAlgorithm tracking_algorithm_;

    std::vector<cv::Ptr<cv::Tracker> > trackers_;
    std::vector<cv::Rect2d> tracking_boxes_;

    ros::Subscriber image_frame_sub_;
    ros::Subscriber bounding_box_sub_;
    ros::Subscriber detection_flag_sub_;

    ros::Publisher visual_detection_image_;
    ros::Publisher visual_detection_bb_;

    std::queue<int> detection_flag_;  
    std::queue<sensor_msgs::Image> video_image_frames_; 
    std::queue<std::vector<darknet_ros_msgs::BoundingBox>> bounding_boxes_matrix_;  

    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void ArrayReceived(const darknet_ros_msgs::BoundingBoxes& msg);
    void ObjectDetected(const std_msgs::Int8& msg);
    void Process();
    bool CheckIfRectMatchesRectVector(cv::Rect2d, std::vector<cv::Rect2d>);
    float ComputeFractionOfIntersection(cv::Rect2d, cv::Rect2d);
    float ComputeRectArea(cv::Rect2d);
  };
}

#endif
