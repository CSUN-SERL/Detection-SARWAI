#ifndef SARWAI_DETECTION_LOGGER_IMAGE_SAVER_H_
#define SARWAI_DETECTION_LOGGER_IMAGE_SAVER_H_

#include <string>
#include <vector>

namespace sarwai {

    class ImageSaver {
    public:
        std::string SaveImage(std::vector<int8_t> image) = 0;
    private:

    };

}

#endif