#include "visual_detection_tracker.h"

namespace sarwai {

  VisualDetectionTracker::VisualDetectionTracker(TrackingAlgorithm alg) {
    this->tracking_algorithm_ = alg;
  }

  void VisualDetectionTracker::TrackFrame(const cv::Mat &image_matrix) {
    for (int i = 0; i < this->trackers_.size(); i++) {
      cv::Mat image_copy = image_matrix.clone();
      cv::Ptr<cv::Tracker> tracker = this->trackers_.at(i);
      cv::Rect2d new_bb;
      bool object_tracked = tracker->update(image_matrix, new_bb);
      if (object_tracked) {
        cv::Point top_left = cv::Point(new_bb.x, new_bb.y);
        cv::Point bottom_right = cv::Point(new_bb.x + new_bb.width,
                                           new_bb.y + new_bb.height);
        cv::rectangle(image_matrix, top_left, bottom_right, 2);
        // cv::rectangle(image_matrix, new_bounding_box, cv::Scalar(255,0,0), 2, 1, 1);
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
    this->trackers_.push_back(new_tracker);
    return;
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
