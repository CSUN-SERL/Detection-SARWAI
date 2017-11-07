#include "visual_detection_tracker.h"
#include "ros/ros.h"

namespace sarwai {

  /*
  Returns true if the detection_bb is found to be "close enough" to any of the existing,
  and currently tracked bounding boxes. This indicates the new detection is already being tracked
  and will be ignored.

  Returns false if there isn't an existing tracking bounding box that matches the detection bounding box
  */
  bool IsRedundantDetection(cv::Rect2d detection_bb, std::vector<cv::Rect2d> tracking_bbs) {
    for (int i = 0; i < tracking_bbs.size(); i++) {
      cv::Rect2d tracking_bb = tracking_bbs.at(i);
      // Compare detection_bb to all bounding boxes in tracking_bbs

      // return true if there's a redundancy detected
    }


    return false;
  }

  VisualDetectionTracker::VisualDetectionTracker(TrackingAlgorithm alg) {
    this->tracking_algorithm_ = alg;
  }

  void VisualDetectionTracker::TrackFrame(const cv::Mat &image_matrix) {
    for (int i = 0; i < this->trackers_.size(); i++) {
      cv::Mat image_copy = image_matrix.clone();
      cv::Ptr<cv::Tracker> tracker = this->trackers_.at(i);
      cv::Rect2d bb = this->tracking_boxes_.at(i);
      bool object_tracked = tracker->update(image_copy, bb);
      if (object_tracked) {
        cv::Point top_left = cv::Point(bb.x, bb.y);
        cv::Point bottom_right = cv::Point(bb.x + bb.width,
          bb.y + bb.height);
          ROS_INFO("%f, %f", bb.x, bb.y);
          cv::rectangle(image_copy, bb, 100, 2, 1);
          cv::imshow("tracking", image_copy);
          cv::waitKey(1);
      }
    }
  }

  void VisualDetectionTracker::AddTrackers(const cv::Mat &image, std::vector<cv::Rect2d> detection_bbs) {
    for (int i = 0; i < detection_bbs.size(); i++) {
      cv::Rect2d det_bb = detection_bbs.at(i);
      if (IsRedundantDetection(det_bb) == false) {
        cv::Ptr<cv::Tracker> new_tracker;
        switch (this->tracking_algorithm_) {
          case TrackingAlgorithm::BOOSTING :
          new_tracker = cv::TrackerBoosting::create(); break;
          case TrackingAlgorithm::MIL :
          new_tracker = cv::TrackerMIL::create(); break;
          case TrackingAlgorithm::KCF :
          new_tracker = cv::TrackerKCF::create(); break;
          case TrackingAlgorithm::TLD :
          new_tracker = cv::TrackerTLD::create(); break;
          case TrackingAlgorithm::MEDIANFLOW :
          new_tracker = cv::TrackerMedianFlow::create(); break;
          case TrackingAlgorithm::GOTURN : 
          new_tracker = cv::TrackerGOTURN::create(); break;
          default:
          return;
        }

        new_tracker->init(image, det_bb);
        cv::namedWindow("tracking", cv::WINDOW_NORMAL);
        this->trackers_.push_back(new_tracker);
        this->tracking_boxes_.push_back(det_bb);
      }

    }
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
