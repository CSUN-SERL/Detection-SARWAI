#ifndef SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_
#define SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_

#include <string>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/core/ocl.hpp>

namespace sarwai {

  enum TrackingAlgorithm {BOOSTING, MIL, KCF, TLD, MEDIANFLOW, GOTURN};
  
  class VisualDetectionTracker {
  public:
    VisualDetectionTracker(TrackingAlgorithm tracking_algorithm);
    void TrackFrame(const cv::Mat &image_matrix);
    void AddTracker(const cv::Mat &image_with_bounding_box, cv::Rect2d);
    
    bool HasActiveTrackers();
    ~VisualDetectionTracker();

  private:
    TrackingAlgorithm tracking_algorithm_;
    // The primary trackers that log data is pulled from
    std::vector<cv::Ptr<cv::Tracker> > trackers_;
    std::vector<cv::Rect2d> tracking_boxes_;
  };
}

#endif
