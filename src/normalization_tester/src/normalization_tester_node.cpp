#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>

void imageDisplay(const sensor_msgs::ImageConstPtr & msg){
  ROS_INFO("Tester received image.");
  cv::namedWindow("winner");
  cv::imshow("winner", cv_bridge::toCvCopy(*msg, (*msg).encoding.c_str())->image);
  cv::waitKey(0);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "normalizertester");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/stereo/image_rect", 1000);
  if(!pub){
    ROS_INFO("Bad publisher.");
  }
  image_transport::Subscriber sub = it.subscribe("/sarwai_detection/normalized_pointcloud", 1000, &imageDisplay);

  cv::Mat image;
  try{
    image = cv::imread("testimage.png", CV_LOAD_IMAGE_GRAYSCALE);
  } catch (const std::exception & e){
    ROS_ERROR("exception: %s", e.what());
  }

  std_msgs::Header head;
  sensor_msgs::ImagePtr thing = cv_bridge::CvImage(head, "mono8", image).toImageMsg();
  sensor_msgs::Image msg = (*thing);
  ROS_INFO("%d", pub.getNumSubscribers());
  while(!pub.getNumSubscribers()){
    ros::Duration(0.1).sleep();
    ROS_INFO("%d", pub.getNumSubscribers());
  }
  pub.publish(msg);
  ROS_INFO("Tester published the image.");

  ros::spin();

  return 0;
}