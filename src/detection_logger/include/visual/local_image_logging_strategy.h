#include "image_logging_strategy.h"

namespace sarwai {

  /**
  *   Implementation of the LoggingStrategy class.
  *   Logs the image and bounding box metadata to a filepath on the local filesystem.
  */
  class LocalImageLoggingStrategy: public ImageLoggingStrategy {
    /**
    *   Initialize this strategy as a LoggingStrategy instance.
    */
    //INIT_IMAGE_STRATEGY(LocalImageLoggingStrategy)
  public:
    void Log(sensor_msgs::Image& image, BoxMetadata boxdata) override;
  private:
    /**
    *   Find the number of the last file saved.
    *   Used to find what the next file's numerical suffix should be.
    *   @param filename Name of the file to parse.
    *   @ret Int containing the number of the file.
    */
    int GetFileNumber(const char * filename) const;

    /**
    *   Verify the filename belongs to an image file.
    *   The only files that this should be checking are image files, with
    *   the exception of the current-directory and parent-directory files.
    *   Returns false if the first character is a period, representing either
    *   a reference to the current or parent directory.
    *   @param filename C-string of the file name to check.
    *   @ret Boolean false if the first character is a period, true otherwise.
    */
    bool IsImageFile(const char * filename) const;
  };

}
