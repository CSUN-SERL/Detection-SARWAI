#include "detection_tracker.h"

namespace sarwai {
  VisualDetectionTracker::TrackFrame(const cv::Mat &image_matrix) {
    for (int i = 0; i < this->trackers_.size(); i++) {
      cv::Mat image_copy = image_matrix.clone();
      std::shared_ptr<cv::Tracker> tracker = this->trackers_.at(i);
      cv::Rect2d new_bounding_box;
      bool object_tracked = tracker->update(image_matrix, bounding_box);
      if (object_tracked) {
        cv::rectangle(image_matrix, bounding_box, Scalar(255,0,0), 2, 1);
      }
    }
  }

  void VisualDetectionTracker::AddTracker(const cv::Mat &image_with_bounding_box, const cv::Rect2d bounding_box) {
    std::shared_ptr<cv::Tracker> new_tracker;
    new_tracker->init(image_with_bounding_box, bounding_box);
    this->trackers_.push_back(new_tracker);
    return;
  }
}