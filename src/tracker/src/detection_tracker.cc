#include "detection_tracker.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace sarwai {

  VisualDetectionTracker::VisualDetectionTracker() {
    this->nh_ = new ros::NodeHandle();
    this->image_frame_sub_ = this->nh_->subscribe(
        "darknet_ros/detection_image", 1000, &VisualDetectionTracker::ImageCallback, this);
    this->bounding_box_sub_ = this->nh_->subscribe(
        "darknet_ros/bounding_boxes", 1000, &VisualDetectionTracker::ArrayReceived, this);
    this->detection_flag_sub_ = this->nh_->subscribe(
        "darknet_ros/found_object", 1000, &VisualDetectionTracker::ObjectDetected, this);
    this->visual_detection_bb_ = this->nh_->advertise<darknet_ros_msgs::BoundingBoxes>(
        "visual_detection_bb", 1000);
    this->visual_detection_image_ = this->nh_->advertise<sensor_msgs::Image>(
        "visual_detection_image", 1000);
    this->visual_detection_flag_ = this->nh_->advertise<std_msgs::Int8>(
        "visual_detection_flag", 1000);

    this->tracking_algorithm_ = TrackingAlgorithm::MEDIANFLOW;
  }

  void VisualDetectionTracker::ArrayReceived(const darknet_ros_msgs::BoundingBoxes &msg) {
    this->bounding_boxes_matrix_.push(msg.boundingBoxes);
  }

  void VisualDetectionTracker::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    this->video_image_frames_.push(*msg);
    Process();
  }

  void VisualDetectionTracker::ObjectDetected(const std_msgs::Int8 &msg) {
    this->detection_flag_.push(msg.data);
  }
  
  /*
   * Process controls the process of receiving messages on incoming ROS topics
   * and the publishing of data after running the tracking redundancy detection system
   */
  void VisualDetectionTracker::Process() {
    // This check only lets this function run if there are also elements in the detection flag and bounding box queues
    if (this->bounding_boxes_matrix_.size() == 0) {
      // If you aren't ready to process all 3 queues, we have to prune them to make sure they don't get backlogged
      if (this->video_image_frames_.size() > 0) {
        this->video_image_frames_.pop();
      }

      if (this->detection_flag_.size() > 0) {
        this->detection_flag_.pop();
      }
    }

    while (this->video_image_frames_.size() > 0 && this->bounding_boxes_matrix_.size() > 0 && this->detection_flag_.size() > 0) {
      std::vector<cv::Rect2d> detection_bbs;
      std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes = bounding_boxes_matrix_.front();
      for (int i = 0; i < bounding_boxes.size(); i++) {
        darknet_ros_msgs::BoundingBox bb = bounding_boxes.at(i);
        cv::Rect2d bb_rect(bb.xmin, bb.ymin, bb.xmax - bb.xmin, bb.ymax - bb.ymin);
        detection_bbs.push_back(bb_rect);
      }

      TrackFrame(cv_bridge::toCvCopy(this->video_image_frames_.front(), sensor_msgs::image_encodings::BGR8)->image, detection_bbs, bounding_boxes);

      std_msgs::Int8 msg;
      darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;

      msg.data = this->detection_flag_.front();
      visual_detection_bb_.publish(out_going_bb);
      visual_detection_image_.publish(this->video_image_frames_.front());
      visual_detection_flag_.publish(msg);
      this->out_going_bb.boundingBoxes.clear();

      this->video_image_frames_.pop();
      this->bounding_boxes_matrix_.pop();
      this->detection_flag_.pop();
    }
  }

  /*
   * TrackFrame receives a image and a vector of boxes demarking visual detection.
   * This is the entry point to the tracking system whose purpose is reducing redundant detections.
   * 
   * Upon receiving a new detection, a tracking box is placed on top of the detection box.
   * With every new frame and new set of detection boxes, the tracking boxes are updated
   * and compared with the set of detection boxes. If the tracking box is determined to come close to
   * being the same as a detection box, we conclude the incoming detection box is redundant, 
   * and we do not need to send it to logging.
   * 
   * TODO: Modify TrackFrame to return a vector of bounding boxes to send to logging
   */
  void VisualDetectionTracker::TrackFrame(const cv::Mat &image_matrix, std::vector<cv::Rect2d> detect_bbs, std::vector<darknet_ros_msgs::BoundingBox> original_bb) {
    cv::Mat image_copy = image_matrix.clone();
    cv::Rect2d bb;
    for (int i = 0; i < this->trackers_.size(); i++) {
      cv::Ptr<cv::Tracker> tracker = this->trackers_.at(i);
      bb = this->tracking_boxes_.at(i);
      if (!CheckIfRectMatchesRectVector(bb, detect_bbs)) {
        ROS_INFO("deleting tracker");
        this->trackers_.erase(this->trackers_.begin() + i);
        this->tracking_boxes_.erase(this->tracking_boxes_.begin() + i);
        continue;
      }
      bool object_tracked = tracker->update(image_copy, bb);

      if (object_tracked) {
        tracking_boxes_.at(i) = bb;
        cv::Point top_left = cv::Point(bb.x, bb.y);
        cv::Point bottom_right = cv::Point(bb.x + bb.width,
                                          bb.y + bb.height);
        cv::rectangle(image_copy, bb, 16711808, 10, 143);
        cv::waitKey(1);
      }
      else {
        ROS_INFO("deleting tracker");
        this->trackers_.erase(this->trackers_.begin() + i);
        this->tracking_boxes_.erase(this->tracking_boxes_.begin() + i);
      }
    }

    AddTrackers(image_matrix, detect_bbs, original_bb);
    cv::imshow("tracking", image_copy);
  }

  void VisualDetectionTracker::AddTrackers(const cv::Mat &image, std::vector<cv::Rect2d> detection_bbs, std::vector<darknet_ros_msgs::BoundingBox> original_bb) {
    for (int i = 0; i < detection_bbs.size(); i++) {
      if (!CheckIfRectMatchesRectVector(detection_bbs.at(i), this->tracking_boxes_)) {
        cv::Ptr<cv::Tracker> new_tracker;
        switch (this->tracking_algorithm_) {
          case TrackingAlgorithm::BOOSTING:
            new_tracker = cv::TrackerBoosting::create();
            break;
          case TrackingAlgorithm::MIL:
            new_tracker = cv::TrackerMIL::create();
            break;
          case TrackingAlgorithm::KCF:
            new_tracker = cv::TrackerKCF::create();
            break;
          case TrackingAlgorithm::TLD:
            new_tracker = cv::TrackerTLD::create();
            break;
          case TrackingAlgorithm::MEDIANFLOW:
            new_tracker = cv::TrackerMedianFlow::create();
            break;
          case TrackingAlgorithm::GOTURN:
            new_tracker = cv::TrackerGOTURN::create();
            break;
          default:
            return;
        }

        new_tracker->init(image, detection_bbs.at(i));
        this->trackers_.push_back(new_tracker);
        this->tracking_boxes_.push_back(detection_bbs.at(i));
        darknet_ros_msgs::BoundingBox temp = original_bb.at(i);
        this->out_going_bb.boundingBoxes.push_back(temp); //Testing
      }
    }
  }

  /*
   * CheckIfRectMatchesRecetVector receives a single bounding box and a vector of bounding boxes.
   * It then runs a set of comparison tests beteween the single box and each element of the vector
   * to determine if the single box is similar (to an extent).
   * 
   * It returns true if the bb matches one of the elements in bbs.
   */
  bool VisualDetectionTracker::CheckIfRectMatchesRectVector(cv::Rect2d bb, std::vector<cv::Rect2d> bbs) {
    for (int i = 0; i < bbs.size(); i++) {
      cv::Rect2d vect_bb = bbs.at(i);
      if (ComputeFractionOfIntersection(bb, vect_bb) > 0.8) {
        return true;
      }
    }

    return false;
  }

    /*  
   * Given two areas A_a, A_b and A_i which is the intersection rect of a and b,
   * this function returns A_i / (A_a + A_b - 2*A_i) which is a value between 0 to 1.
   * It will return 0 if there is no intersection between rects a and b
   */
  float VisualDetectionTracker::ComputeFractionOfIntersection(cv::Rect2d a, cv::Rect2d b) {
    float top_left_x = std::max(a.x, b.x);
    float top_left_y = std::max(a.y, b.y);
    float bot_right_x = std::min((a.x + a.width), (b.x + b.width));
    float bot_right_y = std::min((a.y + a.height), (b.y + b.height));
    
    // Check and see if the two rects a and b are actually intersecting
    if (top_left_x >= bot_right_x || top_left_y > bot_right_y) {
      return 0.0;
    }

    cv::Rect2d intersection(top_left_x, top_left_y, bot_right_x - top_left_x, bot_right_y - top_left_y);
    
    float intersection_area = ComputeRectArea(intersection);
    
    float fraction_of_intersection = intersection_area / (ComputeRectArea(a) + ComputeRectArea(b) - 2 * intersection_area);
    return fraction_of_intersection;
  }

  float VisualDetectionTracker::ComputeRectArea(cv::Rect2d a) {
    float area = a.width * a.height;
    return area;
  }

  VisualDetectionTracker::~VisualDetectionTracker() {}
}
