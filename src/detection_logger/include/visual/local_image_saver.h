#ifndef SARWAI_DETECTION_LOGGER_LOCAL_IMAGE_SAVER_H_
#define SARWAI_DETECTION_LOGGER_LOCAL_IMAGE_SAVER_H_

#include "image_saver.h"

namespace sarwai {

    /**
    *   Implementation of ImageSaver.
    *   Saves an image to the user's local filesystem.
    */
    class LocalImageSaver: public ImageSaver {
    public:
        /**
        *   Save an image to the file system.
        *   @param image sensor_msgs::Image object containing the image to save.
        *   @ret String containing the name of the saved image file.
        */
        std::string SaveImage(sensor_msgs::Image& image) override;
    private:

    };

}

#endif