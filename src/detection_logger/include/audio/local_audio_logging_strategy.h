#ifndef SARWAI_DETECTION_LOGGER_LOCAL_AUDIO_LOGGING_STRATEGY_H_
#define SARWAI_DETECTION_LOGGER_LOCAL_AUDIO_LOGGING_STRATEGY_H_

#include "audio_logging_strategy.h"

namespace sarwai {
  class LocalAudioLoggingStrategy: public AudioLoggingStrategy {
  INIT_AUDIO_STRATEGY(LocalAudioLoggingStrategy);
  public:
    // void Log(Audio data stream type, AudioMetadata boxdata) override;
  };
}

#endif