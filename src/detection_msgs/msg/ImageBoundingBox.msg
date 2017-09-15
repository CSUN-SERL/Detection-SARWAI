std_msgs/Header header
# byte array of raw image
int8[] image 
string detected_class
# percentile rating of detection confidence
float32 confidence 
# coordinates of the top-left corner
int32 box_x_coord
int32 box_y_coord
int32 box_length
int32 box_width
