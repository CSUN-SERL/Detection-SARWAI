#include "visual_detection_tracker.h"
#include "ros/ros.h"

namespace sarwai {

  VisualDetectionTracker::VisualDetectionTracker(TrackingAlgorithm alg) {
    this->tracking_algorithm_ = alg;
  }

  void VisualDetectionTracker::TrackFrame(const cv::Mat &image_matrix) {
    for (int i = 0; i < this->trackers_.size(); i++) {
      cv::Mat image_copy = image_matrix.clone();
      cv::Ptr<cv::Tracker> tracker = this->trackers_.at(i);
      // cv::Rect2d new_bb;
      cv::Rect2d bb = this->tracking_boxes_.at(i);
      bool object_tracked = tracker->update(image_copy, bb);
      if (object_tracked) {
        cv::Point top_left = cv::Point(bb.x, bb.y);
        cv::Point bottom_right = cv::Point(bb.x + bb.width,
          bb.y + bb.height);
          ROS_INFO("%f, %f", bb.x, bb.y);
          // cv::rectangle(image_copy, top_left, bottom_right, 100, 2, 1);
          cv::rectangle(image_copy, bb, 100, 2, 1);
          cv::imshow("tracking", image_copy);
          cv::waitKey(1);
      }
    }
  }

  void VisualDetectionTracker::AddTracker(const cv::Mat &image_with_bounding_box, cv::Rect2d bounding_box) {
    cv::Ptr<cv::Tracker> new_tracker;
    switch (this->tracking_algorithm_) {
      case TrackingAlgorithm::BOOSTING :
      new_tracker = cv::Tracker::create("BOOSTING"); break;
      case TrackingAlgorithm::MIL :
      new_tracker = cv::Tracker::create("MIL"); break;
      case TrackingAlgorithm::KCF :
      new_tracker = cv::Tracker::create("KCF"); break;
      case TrackingAlgorithm::TLD :
      new_tracker = cv::Tracker::create("TLD"); break;
      case TrackingAlgorithm::MEDIANFLOW :
      new_tracker = cv::Tracker::create("TLD"); break;
    }
    new_tracker->init(image_with_bounding_box, bounding_box);
    cv::namedWindow("tracking", cv::WINDOW_NORMAL);
    this->trackers_.push_back(new_tracker);
    this->tracking_boxes_.push_back(bounding_box);
    return;
  }

  bool VisualDetectionTracker::HasActiveTrackers() {
    if (this->trackers_.size() == 0) {
      return false;
    } else {
      return true;
    }
  }

  VisualDetectionTracker::~VisualDetectionTracker() {
    // for (int i = 0; i < this->trackers_.size(); i++) {
    //   cv::Tracker* tmp;
    //   tmp = this->trackers.end();
    //   delete tmp;
    //   this->trackers.push_back();
    // }
  }
}
