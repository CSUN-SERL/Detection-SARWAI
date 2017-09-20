#include "logging_strategy.h"

namespace sarwai {

  class LocalLoggingStrategy: public LoggingStrategy {
  INIT_STRATEGY(LocalLoggingStrategy)
  public:
    void Log(std::vector<int8_t> image, BoxMetadata boxdata) override;
  private:
    int GetFileNumber(const char * filename) const;
  };

}
