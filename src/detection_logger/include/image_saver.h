#ifndef SARWAI_DETECTION_LOGGER_IMAGE_SAVER_H_
#define SARWAI_DETECTION_LOGGER_IMAGE_SAVER_H_

#include <string>
#include <vector>
#include <sensor_msgs/Image.h>

namespace sarwai {

    /**
    *   Interface to log an image. Used with LoggingStrategy class.
    */
    class ImageSaver {
    public:
        /**
        *   Log an image.
        *   @param image sensor_msgs::Image reference that contains the image to log.
        *   @param string containing an identifier to reference the logged image.
        */
        std::string SaveImage(sensor_msgs::Image& image) = 0;
    private:

    };

}

#endif