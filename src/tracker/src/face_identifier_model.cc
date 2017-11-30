#include "face_identifier_model.h"

#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>

#include "detection_frame_id.h"
#include "face_classifier_manager.h"

namespace sarwai {

  FaceIdentifierModel::FaceIdentifierModel() {
    this->similarity_model_ = cv::face::createLBPHFaceRecognizer();
  }

  void FaceIdentifierModel::RunFacePrediction(cv::Mat image) {
    int label;
    double confidence;
    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, cv::COLOR_BGR2GRAY);      

    this->similarity_model_->predict(greyImage, label, confidence);
    std::string str_label = label_map_[label];
    ROS_INFO("label: %s, confidence: %f", str_label.c_str(), confidence);
  }

  void FaceIdentifierModel::ReceiveImage(
    cv::Mat image,
    DetectionFrameId frame_id,
    cv::Rect roi,
    std::vector<cv::Rect> detected_faces) {

    if (this->done_receiving_) {
      assert(0);
    }

    images_.push_back(image);
    initial_rois_.push_back(roi);
    faces_per_image_.push_back(detected_faces);
    frame_ids_.push_back(frame_id);
  }

  void FaceIdentifierModel::DoneReceivingImages() {
    this->done_receiving_ = true;
    TrainModel();
  }

  void FaceIdentifierModel::TrainModel() {
    std::vector<int> labels;
    std::vector<cv::Mat> face_images;

    std::string str_label;

    // Flatten face 2d vector and generate labels
    for (int i = 0; i < this->faces_per_image_.size(); i++) {
      std::vector<cv::Rect> crop_rects = faces_per_image_.at(i);
      for (int j = 0; j < crop_rects.size(); j++) {

        cv::Mat cropped_image(images_.at(i), crop_rects.at(j));
        cv::Mat copied_cropped_image = cropped_image.clone();
        // Concert to grey scale
        cv::Mat greyMat;
        cv::cvtColor(copied_cropped_image, greyMat, cv::COLOR_BGR2GRAY);        
        face_images.push_back(greyMat);

        str_label = FaceClassifierManager::GenerateImageLabel(
          frame_ids_.at(i).DetectionId(), frame_ids_.at(i).FrameId(), j);
        
        int int_label = label_map_.size();
        label_map_[int_label] = str_label;
        labels.push_back(int_label);
      }
    }

    if (labels.size() == 0) {
      return;
    }

    try {
      ROS_INFO("Training %s", str_label.c_str());
      this->similarity_model_->train(face_images, labels);
      ROS_INFO("Done Training %s", str_label.c_str());
    }
    catch (cv::Exception e) {
      // don't stop... belieeeeeving
    }

    this->done_training_ = true;
  }

  bool FaceIdentifierModel::IsDoneTraining() {
    return done_training_;
  }
}
