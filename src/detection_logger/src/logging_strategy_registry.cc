#include "logging_strategy_registry.h"

namespace sarwai {

  std::unique_ptr<LoggingStrategyRegistry> LoggingStrategyRegistry::Instance = std::unique_ptr<LoggingStrategyRegistry>(new LoggingStrategyRegistry);

  bool LoggingStrategyRegistry::Add(std::string name, LoggingStrategy * (*strategygetter)()) {
    std::function<LoggingStrategy*()> getterwrapper = strategygetter;
    strategy_map_.emplace(name, getterwrapper);
  }

  LoggingStrategy* LoggingStrategyRegistry::Get(std::string name) {
    return (strategy_map_[name])(); // TODO: fix the memory leak issue here
  }

}
