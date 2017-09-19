#ifndef SARWAI_DETECTION_SARWAI_DETECTION_LOGGER_LOCAL_BOX_DATA_SAVER_H_
#define SARWAI_DETECTION_SARWAI_DETECTION_LOGGER_LOCAL_BOX_DATA_SAVER_H_

#include <string>
#include "box_metadata.h"

namespace sarwai{

  class LocalBoxDataSaver{
  public:
    LocalBoxDataSaver();
    ~LocalBoxDataSaver();

    void SaveBoxData(const BoxMetadata & boxdata, std::string filename, std::string filepath);
  private:
    std::string FormatOutput(const BoxMetadata & boxdata);
  };

}

#endif
