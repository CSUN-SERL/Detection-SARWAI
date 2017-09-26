#ifndef SARWAI_DETECTION_LOGGER_LOCAL_IMAGE_SAVER_H_
#define SARWAI_DETECTION_LOGGER_LOCAL_IMAGE_SAVER_H_

#include "image_saver.h"

namespace sarwai {

    class LocalImageSaver: public ImageSaver {
    public:
        std::string SaveImage(sensor_msgs::Image& image) override;
    private:

    };

}

#endif