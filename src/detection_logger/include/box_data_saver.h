#ifndef SARWAI_DETECTION-LOGGER_BOX-DATA-SAVER_H_
#define SARWAI_DETECTION-LOGGER_BOX-DATA-SAVER_H_

#include "box_metadata.h"

namespace sarwai{

  class BoxDataSaver{
  public:
    virtual BoxDataSaver();
    virtual ~BoxDataSaver();

    virtual void SaveBoxData(const BoxMetadata & boxdata) = 0;
  private:

  }

}

#endif
