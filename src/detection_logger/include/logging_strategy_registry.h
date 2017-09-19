#ifndef SARWAI_DETECTIONLOGGER_LOGGINGSTRATEGYREGISTRY_H_
#define SARWAI_DETECTIONLOGGER_LOGGINGSTRATEGYREGISTRY_H_

#define REGISTER_STRATEGY(STRATEGYNAME) bool reg##STRATEGYNAME = LoggingStrategyRegistry::Instance->Add(STRATEGYNAME::Classname(), &STRATEGYNAME::get());

#include <string>
#include <function>
#include <unordered_map>
#include <memory>
#include "logging_strategy.h"

namespace sarwai{

  class LoggingStrategyRegistry{
  public:
    bool Add(std::string strategyname, SaverBase * (*strategygetter)());
    static SaverRegistry * Instance();
    LoggingStrategy * get(std::String name);
  private:
    std::unordered_map<std::string, std::function<LoggingStrategy*()>>;
    static std::unique_ptr<LoggingStrategyRegistry> instance;
  }

}

#endif
