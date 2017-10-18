#ifndef SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_
#define SARWAI_DETECTION_IMAGE_DRAW_VISUAL_DETECTION_TRACKER_H_

#include <string>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>
// #include <opencv2/video/tracking.hpp>
#include <tracker.hpp>
#include <opencv2/core/ocl.hpp>

namespace sarwai {
  class VisualDetectionTracker {
  public:
    VisualDetectionTracker(std::string tracking_algorithm);
    void AddTracker(const cv::Mat &image_with_bounding_box);
    void TrackFrame(const cv::Mat &image_matrix);
  private:
    std::string tracking_algorithm_;


    std::vector<std::shared_ptr<cv::Tracker>> trackers_;
    
    cv::VideoCapture video_capture_;
    cv::VideoWriter frame_writer_;
  };
}

#endif