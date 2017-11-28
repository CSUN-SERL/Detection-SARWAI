#include "depth_calculator.h"
#include <cmath>
#include <detection_msgs/ProcessedVisualDetection.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
//#include <pcl_ros/transform.h>


namespace sarwai {

  DepthCalculator::DepthCalculator() {

  }

  DepthCalculator::DepthCalculator(ros::NodeHandle* nh, std::string subtopic, std::string pubtopic): nh_(nh) {
    sub_ = nh_->subscribe(subtopic, 1000, &DepthCalculator::CalculationCallback, this);
    pub_ = nh_->advertise<detection_msgs::ProcessedVisualDetection>(pubtopic, 1000);
  }

  DepthCalculator::~DepthCalculator() {
    //empty
  }

  void DepthCalculator::CalculationCallback(const detection_msgs::DetectionPointCloudConstPtr& msg) {
    darknet_ros_msgs::BoundingBox box = msg->detection.bounding_box;
    ROS_INFO("Box info: x = %ld, xm = %ld, y = %ld, ym = %ld", box.xmin, box.xmax, box.ymin, box.ymax);
    // BOX: 241, 379, 0, 423


    pcl::PCLPointCloud2 pcl_point_cloud;
    pcl_conversions::toPCL(msg->point_cloud, pcl_point_cloud);
    pcl::PointCloud<pcl::PointXYZ> xyzCloud;
    pcl::fromPCLPointCloud2(pcl_point_cloud, xyzCloud);

    float horizontalFOV = 1.047198f;
   
    unsigned imageWidth = msg->detection.image.width;
    unsigned imageHeight = msg->detection.image.height;

    float verticalFOV = horizontalFOV * imageHeight/imageWidth;


    //float angularResolution = 0.0017453293f; // (float) (1.0f * (M_PI/180.0f)); // 1 degree in radians
    float angularResolutionWidth = horizontalFOV / imageWidth;
    float angularResolutionHeight = verticalFOV / imageHeight;
    //float maxAngleWidth = 1.3962634f; //(float) (360.0f * (M_PI/180.0f));
    //float maxAngleHeight = maxAngleWidth;
    Eigen::Affine3f sensorPose(Eigen::Affine3f::Identity());
    pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::CAMERA_FRAME;

    float noiseLevel = 0.0f;
    float minRange = 0.0f;
    int borderSize = 0;

    pcl::RangeImage rangeImage;
    //rangeImage.createFromPointCloud(xyzCloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinateFrame, noiseLevel, minRange, borderSize);
    rangeImage.createFromPointCloud(xyzCloud, angularResolutionWidth, horizontalFOV, verticalFOV, sensorPose, coordinateFrame, noiseLevel, minRange, borderSize);

    // pcl::visualization::RangeImageVisualizer viz("vizor");
    // viz.showRangeImage(rangeImage);
    // viz.spinOnce();
    // pcl_sleep(0.01);

    float distanceAverage = 0;
    unsigned pointTotal = 0;
    for(int y = box.ymin; y < box.ymax; y++) {
      for(int x = box.xmin; x < box.xmax; x++) {

        if(rangeImage.isValid(x, y)){

          //ROS_INFO("getting point %d,%d",x,y);
          pcl::PointWithRange currentPoint = rangeImage.getPoint(x, y);
          //ROS_INFO("done getting point");

          float dist = sqrt(pow(currentPoint.x, 2) + pow(currentPoint.y, 2) + pow(currentPoint.z, 2));
          
          if(dist > 0) {
            //ROS_INFO("dist = %lf",dist);
            distanceAverage += dist;
            ++pointTotal;
          }

        }
      }
    }
    // ROS_INFO("********************************************************");

    distanceAverage /= pointTotal;

    detection_msgs::ProcessedVisualDetection outmsg = msg->detection;
    outmsg.bounding_box.depth = distanceAverage;
    pub_.publish(outmsg);
  }

}
