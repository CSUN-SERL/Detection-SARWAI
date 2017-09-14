#ifndef SARWAI_DETECTION-LOGGER_LOGGING-STRATEGY_H_
#define SARWAI_DETECTION-LOGGER_LOGGING-STRATEGY_H_

#include <vector>
#include "box_metadata.h"

namespace sarwai{

  /**
  *   LoggingStrategy interface, containing one outward facing
  *   function [log(Image data, bounding box data)] that
  *   will save the data to files using an ImageSaver
  *   class and a BoxDataSaver class.
  */
  class LoggingStrategy{
  public:
    LoggingStrategy();
    ~LoggingStrategy();
    
    void Log(std::vector<uint8_t>, BoxMetadata boxdata) = 0;
  private:
    ImageSaver * imagesaverinstance;
    BoxDataSaver * boxdatasaverinstance;
  };

};

#endif
