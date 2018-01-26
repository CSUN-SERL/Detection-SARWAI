// #ifndef SARWAI_DETECTION_LOGGER_LOGGING_STRATEGY_REGISTRY_H_
// #define SARWAI_DETECTION_LOGGER_LOGGING_STRATEGY_REGISTRY_H_

// //#define REGISTER_STRATEGY(STRATEGYNAME) bool reg##STRATEGYNAME = LoggingStrategyRegistry::Instance->Add(STRATEGYNAME::Classname(), &STRATEGYNAME::Get);

// #include <string>
// #include <functional>
// #include <unordered_map>
// #include <memory>
// #include "image_logging_strategy.h"

// namespace sarwai {

//   /**
//   *   Registry for tracking each LoggingStrategy implementation.
//   *
//   *   Contains the macro REGISTER_STRATEGY. This macro should be called at the
//   *   end of the implementation file for each LoggingStrategy implementation.
//   *   This macro calls the Add function in LoggingStrategyRegistry.
//   */
//   class LoggingStrategyRegistry {
//   public:
//     /**
//     *   Add a strategy to the registry. Should ONLY be called using the
//     *   REGISTER_STRATEGY macro.
//     */
//     bool Add(std::string strategyname, ImageLoggingStrategy * (*strategygetter)());

//     /**
//     *   Get the singleton instance of the LoggingStrategyRegistry.
//     */
//     ImageLoggingStrategy * Get(std::string name);

//     /**
//     *   The singleton instance of the LoggingStrategyRegistry.
//     */
//     static std::unique_ptr<LoggingStrategyRegistry> Instance;
//   private:
//     LoggingStrategyRegistry() = default;
//     std::unordered_map<std::string, std::function<ImageLoggingStrategy*()>> strategy_map_;
//   };

// }

// #endif
