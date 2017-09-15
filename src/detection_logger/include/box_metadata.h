#ifndef SARWAI_DETECTION-LOGGER_BOX-METADATA_H_
#define SARWAI_DETECTION-LOGGER_BOX-METADATA_H_

#include <string>

namespace sarwai{
  struct BoxMetadata{
    std::string object_class;
    double confidence_rating;
    int timestamp;
    int left_x_coord;
    int top_y_coord;
    int box_width;
    int box_height;
  }
}

#endif
