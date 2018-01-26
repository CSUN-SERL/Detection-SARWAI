#ifndef SARWAI_DETECTION_LOGGER_LOCAL_AUDIO_LOGGING_STRATEGY_H_
#define SARWAI_DETECTION_LOGGER_LOCAL_AUDIO_LOGGING_STRATEGY_H_

#include <string>
#include "audio_logging_strategy.h"

namespace sarwai {
  class LocalAudioLoggingStrategy: public AudioLoggingStrategy {
  //INIT_AUDIO_STRATEGY(LocalAudioLoggingStrategy);
  public:
    LocalAudioLoggingStrategy() = default;
    void Log(std::string audio_file_name, struct AudioMetadata boxdata) override;
  };
}

#endif