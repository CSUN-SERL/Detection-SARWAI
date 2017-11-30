#ifndef SARWAI_DETECTION_TRACKER_DETECTION_FRAME_ID_
#define SARWAI_DETECTION_TRACKER_DETECTION_FRAME_ID_

namespace sarwai {

  class DetectionFrameId {
    public:
    DetectionFrameId();
    int DetectionId();
    int FrameId();
    void IncFrame();
    private:
    static int detection_id_count_;
    int detection_id_;
    int nth_frame_;
  };
}

#endif