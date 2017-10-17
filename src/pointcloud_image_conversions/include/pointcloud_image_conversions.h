#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/io/pcd_io.h>

namespace sarwai {
  namespace pc_conversions {
    sensor_msgs::Image extractImageFromPointCloud2(const sensor_msgs::PointCloud2& cloud);
  }
}