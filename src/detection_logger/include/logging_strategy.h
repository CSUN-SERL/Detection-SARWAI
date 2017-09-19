#ifndef SARWAI_DETECTIONLOGGER_LOGGINGSTRATEGY_H_
#define SARWAI_DETECTIONLOGGER_LOGGINGSTRATEGY_H_

#define INIT_STRATEGY(STRATEGYNAME) public: static std::string Classname() {return #STRATEGYNAME;} static LoggingStrategy * get(){return new STRATEGYNAME;}

#include <vector>
#include <string>
#include "box_metadata.h"


namespace sarwai {
  /**
  *   LoggingStrategy interface, containing one outward facing
  *   function [log(Image data, bounding box data)] that
  *   will save the data to files using an ImageSaver
  *   class and a BoxDataSaver class.
  */
  class LoggingStrategy {
  public:
    virtual void Log(std::vector<uint8_t> image, BoxMetadata boxdata) = 0;
  };

};

#endif
