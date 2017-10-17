#include "pointcloud_image_conversions.h"

namespace sarwai {
  namespace pc_conversions {

    sensor_msgs::Image extractImageFromPointCloud2(const sensor_msgs::PointCloud2& cloud) {
      sensor_msgs::Image ret;
      pcl::toROSMsg(cloud, ret);
      ret.header = cloud.header;
      return ret;
    }

  }
}