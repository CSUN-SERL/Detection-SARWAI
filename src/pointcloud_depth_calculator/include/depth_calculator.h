#ifndef SARWAI_DEPTH_CALCULATOR_H_
#define SARWAI_DEPTH_CALCULATOR_H_

#include <string>
#include <ros/ros.h>
#include <detection_msgs/DetectionPointCloud.h>

namespace sarwai {

  class DepthCalculator {
  public:
    DepthCalculator();
    DepthCalculator(ros::NodeHandle* nh, std::string subtopic, std::string pubtopic);
    ~DepthCalculator();

    void CalculationCallback(const detection_msgs::DetectionPointCloudConstPtr& msg);
  private:
    ros::NodeHandle* nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
  };

}

#endif