/**
*   LoggingStrategy class, containing one outward facing
*   function [log(Image data, bounding box data)] that
*   will save the data to files using an ImageSaver
*   class and a BoxDataSaver class.
*/
#include <vector>

namespace sarwai{

  class LoggingStrategy{
  public:
    LoggingStrategy();
    
    log(std::vector<uint8_t>, BoxMetadata boxdata);
  private:
    
  };

};
