#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h> 
#include <std_msgs/Int8.h>
#include <vector>
#include <queue>
#include <string>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>
#include <image_draw/info.h>
//#include "publish.h"

std::vector<int> information;

std::queue<std::vector<int> > cordinates; //Queue of information vector
std::queue<cv::Mat> vid; //Video
std::queue<int> check; //0 and 1s

int count=0;


void run_image_process();
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void array_recived(const darknet_ros_msgs::BoundingBoxes& msg);
void object_detected(const std_msgs::Int8& msg); //If object dected publishes 1. else pulishes 0




/********************************* MAIN ***************************************************************/

int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "image_listener");

  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("darknet_ros/detection_image", 1, imageCallback);
  ros:: Subscriber sub2 = nh.subscribe("darknet_ros/bounding_boxes", 1000, array_recived);
  ros:: Subscriber sub3 = nh.subscribe("darknet_ros/found_object", 1000, object_detected);
  //ros:: Publisher pub2 = nh.advertise<image_draw::info>("custom_image", 1000 );


  ros::spin();
  cv::destroyWindow("view");
}


/********************************************************************************************************/



void imageCallback(const sensor_msgs::ImageConstPtr& msg) //Video CAll Back Function
{


      cv_bridge::CvImagePtr cam_image; //Decalring an image object

      try {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      cv::Mat mat = cam_image->image;

      vid.push(mat); //Insert vidoe in queue
      run_image_process();  //Passing the image insdie the run_image_function


}


/**************************************************************************************************************/


//Subscribes to yolo Video image
void array_recived(const darknet_ros_msgs::BoundingBoxes& msg){


    for(int i=0; i<msg.boundingBoxes.size(); i++){
        if(msg.boundingBoxes[i].Class == "person"){  //Filters people only
                  
            information.push_back(msg.boundingBoxes[i].xmin);   //xmin is pushed to vector
            information.push_back(msg.boundingBoxes[i].ymin);   //ymin is pushed to vector
            information.push_back(msg.boundingBoxes[i].xmax);   //xmin is pushed to vector
            information.push_back(msg.boundingBoxes[i].ymax);   //xmax i pushed to vector
            //information.push_back(msg.boundingBoxes[i].probability);
            

            cordinates.push(information); // Vector of information is pushed into corinates QUEUE
        }

        information.clear();  //Vector is cleared after each iteration 
        
    }
    
  
}


/*******************************************************************************************************************/


//Subscribes to YOLO object_deteted
void object_detected(const std_msgs::Int8& msg){
    
    check.push(msg.data);   //Sequence of 0s and 1s are inserted into check QUEUE


}


/*******************************************************************************************************************/



void run_image_process(){

    if(!vid.empty() &&  !check.empty()){

        if(check.front() == 1){

            if(!cordinates.empty()){

                cv::Point topLeftCorner = cv::Point(cordinates.front()[0], cordinates.front()[1]);
                cv::Point botRightCorner = cv::Point(cordinates.front()[2], cordinates.front()[3]); 
                cv::rectangle(vid.front(), topLeftCorner, botRightCorner, 2);         
                
                sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", vid.front()).toImageMsg();
                image_draw::info msg2;
                msg2.im = *im_msg;
                msg2.xmin = cordinates.front()[0];
                msg2.ymin = cordinates.front()[1];
                msg2.xmax = cordinates.front()[2];
                msg2.ymax = cordinates.front()[3];

                //pub2.publish(msg2);

                cordinates.pop();

             }

        //std::string s = std::to_string(count++);
        //cv::imwrite( s + ".jpg", vid.front());

        }

        vid.pop();
        check.pop();
     }

}


