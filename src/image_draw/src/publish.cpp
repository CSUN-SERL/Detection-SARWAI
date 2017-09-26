#include "publish.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

publish::publish(){

    pub = nh.advertise<image_draw::info>("custom_image", 1000 );

}

void publish::publish_info(image_draw::info msg){


    pub.publish(msg);



}

