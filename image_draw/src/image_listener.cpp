#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h> 
#include <std_msgs/Int8.h>
#include <vector>
#include <queue>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>


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
    

  information.push_back(10);

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("darknet_ros/detection_image", 1, imageCallback);
  ros:: Subscriber sub2 = nh.subscribe("darknet_ros/bounding_boxes", 1000, array_recived);
  ros:: Subscriber sub3 = nh.subscribe("darknet_ros/found_object", 1000, object_detected);

      

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



void array_recived(const darknet_ros_msgs::BoundingBoxes& msg){

    

    for(int i=0; i<msg.boundingBoxes.size(); i++){
        if(msg.boundingBoxes[i].Class == "person"){
                  
            information.push_back(msg.boundingBoxes[i].xmin);
            information.push_back(msg.boundingBoxes[i].ymin);
            information.push_back(msg.boundingBoxes[i].xmax);
            information.push_back(msg.boundingBoxes[i].ymax);

            cordinates.push(information); // Queue
        }

        information.clear();
        
    }
    
  
}


/*******************************************************************************************************************/



void object_detected(const std_msgs::Int8& msg){
    
    check.push(msg.data);


}


/*******************************************************************************************************************/



void run_image_process(){

    if(!vid.empty() &&  !check.empty()){

        if(check.front() == 1){

            if(!cordinates.empty()){

                cv::Point topLeftCorner = cv::Point(cordinates.front()[0], cordinates.front()[1]);
                cv::Point botRightCorner = cv::Point(cordinates.front()[2], cordinates.front()[3]); 
                cv::rectangle(vid.front(), topLeftCorner, botRightCorner, 2);         
                cordinates.pop();

             }

        }

      try
      {
        cv::imshow("view", vid.front());
        cv::waitKey(30);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from  to 'bgr8'.");
      }




        vid.pop();
        check.pop();
     }


     //cv::imwrite( str + ".jpg", vid.front());


}



/*






      try
      {
        cv::imshow("view", vid.front());
        cv::waitKey(30);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from  to 'bgr8'.");
      }







        ROS_INFO_STREAM("CHECKKKK"<<check.size());
        ROS_INFO_STREAM("VIDDDDD"<<vid.size());
        ROS_INFO_STREAM("CORDDDDD"<<cordinates.size());




















*/





