#ifndef SARWAI_DETECTION_LOGGER_IMAGE_SAVER_H_
#define SARWAI_DETECTION_LOGGER_IMAGE_SAVER_H_

#include <string>
#include <vector>
#include <sensor_msgs/Image.h>

namespace sarwai {

    class ImageSaver {
    public:
        std::string SaveImage(sensor_msgs::Image& image) = 0;
    private:

    };

}

#endif