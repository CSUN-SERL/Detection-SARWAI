#include "ros/ros.h"
#include "detection_tracker.h"



/********************************* MAIN ***************************************************************/

int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "image_track");
  sarwai::VisualDetectionTracker track;
  
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
