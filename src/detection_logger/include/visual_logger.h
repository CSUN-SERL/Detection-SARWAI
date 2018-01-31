#ifndef SARWAI_DETECTION_LOGGER_VISUAL_LOGGER_H_
#define SARWAI_DETECTION_LOGGER_VISUAL_LOGGER_H_

#include <string>

#include <opencv2/opencv.hpp>

#include "logger.h"
#include "visual_detection_data.h"

namespace sarwai {
  class VisualLogger : public Logger {
    public:
    VisualLogger(std::string base_filepath);
    VisualLogger();
    // Returns saved image filename
    std::string Log(cv::Mat image, struct VisualDetectionData detection_data);

    protected:
    std::string GenerateStringCSV(struct VisualDetectionData data, std::string image_filename);
    
    void LocalSaveDetectionData(struct VisualDetectionData data, std::string saved_image_filename);
    std::string SaveImage(cv::Mat image);

    std::string GenerateImageFilename();
    std::string GenerateTextLogFilename();

    int ImageSuffixIterator();

    private:

    int image_suffix_iterator_;
  };
}

#endif