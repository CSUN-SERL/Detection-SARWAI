#include"detection_frame_id.h"

namespace sarwai {

  int DetectionFrameId::detection_id_count_ = 0;

  DetectionFrameId::DetectionFrameId() {
    DetectionFrameId::detection_id_count_ += 1;

    detection_id_ = DetectionFrameId::detection_id_count_;
    nth_frame_ = 0;
  }

  int DetectionFrameId::DetectionId() {
    return detection_id_;
  }

  int DetectionFrameId::FrameId() {
    return nth_frame_;
  }

  void DetectionFrameId::IncFrame() {
    nth_frame_ += 1;
  }
}