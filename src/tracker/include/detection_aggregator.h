#ifndef SARWAI_DETECTION_TRACKER_DETECTION_AGGREGATOR_H_
#define SARWAI_DETECTION_TRACKER_DETECTION_AGGREGATOR_H_

#include "detection_tracker.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>

namespace sarwai {

  class DetectionAggregation {
    public:
    cv::Ptr<cv::Tracker> tracker;
    cv::Rect bb;
    DetectionFrameId* id;
    private:
  };
}

#endif