#ifndef SARWAI_DETECTION-SARWAI_DETECTION-LOGGER_LOCAL-BOX-DATA-SAVER_H_
#define SARWAI_DETECTION-SARWAI_DETECTION-LOGGER_LOCAL-BOX-DATA-SAVER_H_

#include <string>
#include "box_metadata.h"

namespace sarwai{

  class LocalBoxDataSaver{
  public:
    virtual LocalBoxDataSaver();
    virtual ~LocalBoxDataSaver();

    virtual void SaveBoxData(const BoxMetadata & boxdata, std::string filename, std::string filepath);
  private:
    virtual std::string FormatOutput(const BoxMetadata & boxdata);
  }

}

#endif
