#ifndef SARWAI_DETECTION_LOGGER_BOX_METADATA_H_
#define SARWAI_DETECTION_LOGGER_BOX_METADATA_H_

#include <string>

namespace sarwai {
  struct BoxMetadata {
    std::string object_class;
    double confidence_rating;
    int timestamp;
    int left_x_coord;
    int top_y_coord;
    int box_width;
    int box_height;
    unsigned robot_id;
  };
}

#endif