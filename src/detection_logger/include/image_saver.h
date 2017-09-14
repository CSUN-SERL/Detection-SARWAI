#ifndef SARWAI_DETECTION-SARWAI_DETECTION-LOGGER_IMAGE-SAVER_H_
#define SARWAI_DETECTION-SARWAI_DETECTION-LOGGER_IMAGE-SAVER_H_

#include <string>
#include <vector>

namespace sarwai{

  class ImageSaver{
  public:
    virtual ImageSaver();
    virtual ~ImageSaver();

    virtual std::string SaveImage(const std::vector<uint8_t> & image) = 0;
  private:


  }

}

#endif
