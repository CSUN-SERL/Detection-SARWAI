#ifndef SARWAI_DETECTION_LOGGER_LOGGING_STRATEGY_REGISTRY_H_
#define SARWAI_DETECTION_LOGGER_LOGGING_STRATEGY_REGISTRY_H_

#define REGISTER_STRATEGY(STRATEGYNAME) bool reg##STRATEGYNAME = LoggingStrategyRegistry::Instance->Add(STRATEGYNAME::Classname(), &STRATEGYNAME::get);

#include <string>
#include <functional>
#include <unordered_map>
#include <memory>
#include "logging_strategy.h"

namespace sarwai {

  class LoggingStrategyRegistry {
  public:
    bool Add(std::string strategyname, LoggingStrategy * (*strategygetter)());
    static std::unique_ptr<LoggingStrategyRegistry> Instance;
    LoggingStrategy * Get(std::string name);
  private:
    std::unordered_map<std::string, std::function<LoggingStrategy*()>> strategy_map_;
  };

}

#endif
