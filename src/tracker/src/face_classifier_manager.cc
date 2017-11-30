#include "face_classifier_manager.h"
#include "ros/ros.h"

namespace sarwai {

FaceClassifierManager::FaceClassifierManager() {
  this->front_face_cascade_ = new cv::CascadeClassifier();

  this->front_face_cascade_->load("/home/kenny/programming/sarwai/detection/haarcascade_frontalface_alt2.xml");
  ROS_INFO("classifier model loaded");
}

FaceClassifierManager::~FaceClassifierManager() {
  delete this->front_face_cascade_;
}

std::vector<cv::Rect> FaceClassifierManager::RunFacialDetection(
  cv::Mat image, DetectionFrameId image_id, cv::Rect roi) {

  std::vector<cv::Rect> classified_faces;
  this->front_face_cascade_->detectMultiScale(
    image,
    classified_faces,
    1.1,
    3,
    0,
    cv::Size(50,50)
  );

  if (face_identifiers_.find(image_id.DetectionId()) == face_identifiers_.end()) {
    // New detection
    face_identifiers_[image_id.DetectionId()];
  }
  
  FaceIdentifierModel receiving_model = face_identifiers_[image_id.DetectionId()];

  receiving_model.ReceiveImage(image, image_id, roi, classified_faces);
  face_identifiers_[image_id.DetectionId()] = receiving_model;

  return classified_faces;
}

void FaceClassifierManager::DeactivateModel(int detection_id) {

  FaceIdentifierModel model = face_identifiers_[detection_id];
  model.DoneReceivingImages();
  face_identifiers_[detection_id] = model;
}


std::string FaceClassifierManager::GenerateImageLabel(
  int detection_id,
  int nth_frame,
  int classification_id) {
    std::string image_label = std::to_string(detection_id) +
    "." + std::to_string(nth_frame) + "." + std::to_string(classification_id);
    return image_label;
}

void FaceClassifierManager::FindDoppelganger(cv::Mat image, cv::Rect roi) {
  std::map<int, FaceIdentifierModel>::iterator iter;
  for(iter = face_identifiers_.begin(); iter != face_identifiers_.end(); ++iter) {
    int key =  iter->first;
    FaceIdentifierModel model = face_identifiers_[key];
    if (model.IsDoneTraining()) {
      model.RunFacePrediction(image);
    }
    
  }
}

}