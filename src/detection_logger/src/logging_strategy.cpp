#include "logging_strategy.h"

namespace sarwai{

  LoggingStrategy::LoggingStrategy(){
    // TODO: populate the private members somehow. Or save that
    //       for implementations.
  }

  LoggingStrategy::~LoggingStrategy(){
    delete imagesaverinstance;
    delete boxdatasaverinstance;
  }

};
