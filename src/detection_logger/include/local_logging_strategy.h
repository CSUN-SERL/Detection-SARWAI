#include "logging_strategy.h"

namespace sarwai {

  /**
  *   Implementation of the LoggingStrategy class.
  *   Logs the image and bounding box metadata to a filepath on the local filesystem.
  */
  class LocalLoggingStrategy: public LoggingStrategy {
    /**
    *   Initialize this strategy as a LoggingStrategy instance.
    */
    INIT_STRATEGY(LocalLoggingStrategy)
  public:
    void Log(sensor_msgs::Image& image, BoxMetadata boxdata) override;
  private:
    /**
    *   Find the number of the last file saved.
    *   Used to find what the next file's numerical suffix should be.
    */
    int GetFileNumber(const char * filename) const;
    bool IsImageFile(const char * filename) const;
  };

}
