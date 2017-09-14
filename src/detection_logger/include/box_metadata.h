#ifndef SARWAI_DETECTION-LOGGER_BOX-METADATA_H_
#define SARWAI_DETECTION-LOGGER_BOX-METADATA_H_

// TODO: Delete this whole file when integrating with the Matt and Aren's part.
// This will all be contained in the message we get from the rostopic. Silly me

namespace sarwai{
  struct BoxMetadata{
    double confidence_rating;
    int timestamp;
    int left_x_coord;
    int top_y_coord;
    int box_width;
    int box_height;
  }
}

#endif
