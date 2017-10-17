#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <detection_msgs/DetectionPointImage.h>
#include <detection_msgs/DetectionPointCloud.h>

bool ensureOnce = true;
ros::Publisher pub;
ros::Publisher imgpub;

void imageDisplay(const detection_msgs::DetectionPointImageConstPtr& msg) {
// void imageDisplay(const sensor_msgs::ImageConstPtr& msg) {
  // if(ensureOnce) {
    // ensureOnce = false;
    ROS_INFO("Tester received image.");
    // cv::namedWindow("winner");
    // cv::imshow("winner", cv_bridge::toCvCopy(msg->point_image, msg->point_image.encoding.c_str())->image);
    // imshow("winner", cv_bridge::toCvCopy(*msg, msg->encoding.c_str())->image);
    // cv::waitKey(1);
    imgpub.publish(msg->point_image);
  // }
}

void constructDummyMessage(const sensor_msgs::PointCloud2ConstPtr& msg) {
  detection_msgs::DetectionPointCloud outmsg;
  detection_msgs::ProcessedVisualDetection fakedetec;
  darknet_ros_msgs::BoundingBox bb;
  bb.Class = "Person";
  bb.probability = 0.7854;
  bb.xmin = 15;
  bb.xmax = 65;
  bb.ymin = 15;
  bb.ymax = 107;

  fakedetec.bounding_box = bb;
  outmsg.detection = fakedetec;
  outmsg.point_cloud = *msg;

  ROS_INFO("density = %d", msg->is_dense);
  
  while(!pub.getNumSubscribers()) {
    ros::Duration(0.1).sleep();
  }
  pub.publish(outmsg);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "normalizertester");
  ros::NodeHandle nh;
  // image_transport::ImageTransport it(nh);
  // image_transport::Publisher pub = it.advertise("/stereo/image_rect", 1000);
  pub = nh.advertise<detection_msgs::DetectionPointCloud>("/sarwai_detection/detection_pointcloud", 1000);
  ros::Publisher pubcloud = nh.advertise<sensor_msgs::PointCloud2>("pointcloudtopic", 1000);
  ros::Publisher imgpub = nh.advertise<sensor_msgs::Image>("topicthingy", 1000);
  // if(!pub){
    // ROS_INFO("Bad publisher.");
  // }
  // image_transport::Subscriber sub = it.subscribe("/sarwai_detection/normalized_pointcloud", 1000, &imageDisplay);
  ros::Subscriber sub = nh.subscribe("/sarwai_detection/detection_pointimage", 1000, &imageDisplay);
  ros::Subscriber cloudsub = nh.subscribe("/stereo/points2", 1000, &constructDummyMessage);

  /**/
  // cv::Mat image;
  // try{
    // image = cv::imread("testimage.png", CV_LOAD_IMAGE_GRAYSCALE);
  // } catch (const std::exception & e){
    // ROS_ERROR("exception: %s", e.what());
  // }

  // std_msgs::Header head;
  // sensor_msgs::ImagePtr thing = cv_bridge::CvImage(head, "mono8", image).toImageMsg();
  // sensor_msgs::Image msg = (*thing);
  // ROS_INFO("%d", pub.getNumSubscribers());
  // while(!pub.getNumSubscribers()){
    // ros::Duration(0.1).sleep();
    // ROS_INFO("%d", pub.getNumSubscribers());
  // }
  // pub.publish(msg);
  // ROS_INFO("Tester published the image.");

  //Construct dummy data message
  // detection_msgs::DetectionPointCloud outmsg;
  // detection_msgs::ProcessedVisualDetection fakedetec;
  // darknet_ros_msgs::BoundingBox bb;
  // bb.Class = "Person";
  // bb.probability = 0.7854;
  // bb.xmin = 15;
  // bb.xmax = 65;
  // bb.ymin = 15;
  // bb.ymax = 107;

  // fakedetec.bounding_box = bb;
  // ROS_INFO("waiting for message");
  // boost::shared_ptr<sensor_msgs::PointCloud2 const> pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/stereo/points2", nh);
  // ROS_INFO("got message");

  // outmsg.detection = fakedetec;
  // outmsg.point_cloud = *pc;

  // ROS_INFO("density = %d", pc->is_dense);
  
  // while(!pub.getNumSubscribers()) {
  //   ros::Duration(0.1).sleep();
  // }
  // pub.publish(outmsg);
  // pubcloud.publish(*pc);

  ros::spin();

  ros::Duration(10).sleep();

  /** /

  boost::shared_ptr<sensor_msgs::PointCloud2 const> pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/stereo/points2", nh);
  sensor_msgs::Image testimg;
  testimg.header = pc->header;
  testimg.height = pc->height;
  testimg.width = pc->width;
  testimg.is_bigendian = pc->is_bigendian;
  // testimg.step = pc->row_step;
  // testimg.data = pc->data;
  ROS_INFO("size = %d, width = %d, height = %d", pc->point_step, pc->width, pc->height);

  testimg.encoding = "bgr8";
  ros::Publisher imgpub = nh.advertise<sensor_msgs::Image>("testimagetopic", 1000);
  while(!imgpub.getNumSubscribers()) {
    ros::Duration(0.1).sleep();
  }
  imgpub.publish(testimg);

  ros::Duration(10).sleep();
  /**/
  return 0;
}