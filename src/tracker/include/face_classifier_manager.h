#ifndef SARWAI_DETECTION_TRACKER_FACE_CLASSIFIER_MANAGER_H_
#define SARWAI_DETECTION_TRACKER_FACE_CLASSIFIER_MANAGER_H_

#include <vector>
#include <string>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>

#include "detection_frame_id.h"
#include "face_identifier_model.h"

namespace sarwai {

  class FaceClassifierManager {
    public:

    FaceClassifierManager();
    ~FaceClassifierManager();

    static std::string GenerateImageLabel(int detection_id, int nth_frame, int classification_id);

    std::vector<cv::Rect> RunFacialDetection(cv::Mat, DetectionFrameId, cv::Rect);

    void DeactivateModel(int detection_id);

    void FindDoppelganger(cv::Mat, cv::Rect);

    private:
    
    cv::CascadeClassifier* front_face_cascade_;

    std::map<int, FaceIdentifierModel> face_identifiers_;
  };
}

#endif