#include "image_draw/info.h"
#include <ros/ros.h>

class publish{

private:
	ros::Publisher pub;
	ros::NodeHandle n;
    

public:
	publish();
    void publish_info(image_draw::info msg);

}
