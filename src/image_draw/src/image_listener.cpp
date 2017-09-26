#include "ros/ros.h"
#include "image_bounding_box_merger.h"


/********************************* MAIN ***************************************************************/

int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "image_listener");
  sarwai::ImageBoundingBoxMerger merger;
  
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
