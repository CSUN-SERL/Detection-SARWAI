#ifndef SARWAI_DETECTION_LOGGER_AUDIO_LOGGER_H_
#define SARWAI_DETECTION_LOGGER_AUDIO_LOGGER_H_

#include <string>

#include "logger.h"
#include "audio_detection_data.h"

namespace sarwai {
  class AudioLogger : public Logger {
    public:
    AudioLogger();
    AudioLogger(std::string base_filepath);
    void Log(struct AudioDetectionData);
    void LocalSaveDetectionData(struct AudioDetectionData);
    
    protected:
    std::string GenerateStringCSV(struct AudioDetectionData);
  };
}

#endif