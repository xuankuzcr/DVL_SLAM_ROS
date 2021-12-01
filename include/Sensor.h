//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_SENSOR_H
#define DVL_SLAM_MODIFY_SENSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "Frame.h"
#include <image_transport/image_transport.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

class Sensor{
public:
  Sensor(Config& config);
  ~Sensor();

  virtual void data2Frame(Frame& frame) = 0;

  void publishImg(cv::Mat image);
  void publishTransform(Sophus::SE3f input);

  bool IsLidarSubscribed();
  bool IsVisionSubscribed();

protected:
  Config& config_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud_;
  cv::Mat inputImg_;

  bool lidarFlag_;
  bool imgFlag_;

  float minZ;
  float maxZ;

};

#endif //DVL_SLAM_MODIFY_SENSOR_H
