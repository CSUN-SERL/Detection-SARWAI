#include <string>
#include <fstream>

#include <boost/filesystem.hpp>

#include "visual_logger.h"

namespace sarwai {
  
  VisualLogger::VisualLogger(std::string base_filepath) : Logger(base_filepath) {
    image_suffix_iterator_ = 1;
  }  

  std::string VisualLogger::Log(cv::Mat image, struct VisualDetectionData data) {
    std::string image_filename = SaveImage(image);
    LocalSaveDetectionData(data, image_filename);
    return image_filename;
  }

  std::string VisualLogger::SaveImage(cv::Mat image) {
    std::string full_image_path = log_filepath_ + boost::filesystem::path::preferred_separator + GenerateImageFilename();
    cv::imwrite(full_image_path, image);
    return full_image_path;
  }

  void VisualLogger::LocalSaveDetectionData(struct VisualDetectionData data, std::string saved_image_filename) {
      std::string csv_line = GenerateStringCSV(data, saved_image_filename);
      std::string full_text_log_filepath = log_filepath_ + boost::filesystem::path::preferred_separator + "visual-detections.csv";
      
      std::ofstream outfile(full_text_log_filepath, std::ofstream::app | std::ofstream::out);
      if(!outfile.is_open()) {
        std::cerr << "Couldn't open 'output.txt'" << std::endl;
        return;
      }

      outfile << csv_line;
      outfile.close();
  }

  std::string VisualLogger::GenerateStringCSV(struct VisualDetectionData data, std::string saved_image_filename) {
    std::stringstream csv_line;
    csv_line <<
    // "visual" field added for integration to the socketio server
    "visual" << "," <<
    data.object_class << "," <<
    data.robot_id << "," <<
    data.confidence_rating << "," <<
    data.timestamp << "," <<
    data.left_x_coord << "," <<
    data.top_y_coord << "," <<
    data.box_width << "," <<
    data.box_height << "," <<
    saved_image_filename <<
    "\n";

    return csv_line.str();
  }

  std::string VisualLogger::GenerateImageFilename() {
    std::string image_name = "image-" + std::to_string(ImageSuffixIterator()) + ".png";
    return image_name;
  }

  int VisualLogger::ImageSuffixIterator() {
    // increment after return
    return image_suffix_iterator_++;
  }
}