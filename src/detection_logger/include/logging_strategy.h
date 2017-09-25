#ifndef SARWAI_DETECTION_LOGGER_LOGGING_STRATEGY_H_
#define SARWAI_DETECTION_LOGGER_LOGGING_STRATEGY_H_

#define INIT_STRATEGY(STRATEGYNAME) public: static std::string Classname() {return #STRATEGYNAME;} static LoggingStrategy * Get(){return new STRATEGYNAME;}

#include <vector>
#include <stdint.h>
#include "box_metadata.h"


namespace sarwai {
  /**
  *   LoggingStrategy interface, containing one outward facing
  *   function [Log(Image data, bounding box data)] that
  *   will save the data to files using an ImageSaver
  *   class and a BoxDataSaver class.
  *
  *   Contains the macro INIT_STRATEGY. This macro should be called at the
  *   beginning of an implementation of this class. It adds two public
  *   static functions: Classname(), which returns the name of the strategy,
  *   and Get(), which returns a new instance of the child class. These
  *   functions should ONLY be used in the REGISTER_STRATEGY macro in
  *   LoggingStrategyRegistry.
  */
  class LoggingStrategy {
  public:

    /**
    *   Log the image and metadata of the bounding box within the image.
    *   @param image int8_t vector containing the pixel data of the image.
    *   @param boxdata Metadata of the bounding box in the image.
    */
    virtual void Log(std::vector<int8_t> image, BoxMetadata boxdata) = 0;
  };

};

#endif
