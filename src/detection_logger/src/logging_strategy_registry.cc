#include "logging_strategy_registry.h"

namespace sarwai {

  std::unique_ptr<LoggingStrategyRegistry> LoggingStrategyRegistry::Instance = std::unique_ptr<LoggingStrategyRegistry>(new LoggingStrategyRegistry);

  bool LoggingStrategyRegistry::Add(std::string name, ImageLoggingStrategy * (*strategygetter)()) {
    std::function<ImageLoggingStrategy*()> getterwrapper = strategygetter;
    strategy_map_.emplace(name, getterwrapper);
  }

  ImageLoggingStrategy* LoggingStrategyRegistry::Get(std::string name) {
    return (strategy_map_[name])(); // TODO: fix the memory leak issue here
  }

}
