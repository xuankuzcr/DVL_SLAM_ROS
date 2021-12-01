//
// Created by jongsik on 20. 12. 18..
//

#ifndef DVL_SLAM_MODIFY_LOGGER_H
#define DVL_SLAM_MODIFY_LOGGER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "PinholeModel.h"
#include "Config.h"
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>

class Logger{
public:
  Logger(Config& config);
  ~Logger();

  void PushBackOdometryResult(pcl::PointXYZ odometryPoint);
  void PushBackColorMapResult(pcl::PointCloud<pcl::PointXYZRGB> mapCloud, Sophus::SE3f T);
  void PushBackNonColorMapResult(pcl::PointCloud<pcl::PointXYZRGB> mapCloud, Sophus::SE3f T);

  void SaveOdometryResult();
  void SaveMapResult();

  void PublishImg(cv::Mat image);
  void PublishTransform(Sophus::SE3f input);

  void PublishOdometryPoint();
  void PublishColorMapPointCloud();
  void PublishNonColorMapPointCloud();

private:
  Config &config_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr odometryPointCloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapColorPointCloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mapNonColorPointCloud_;

  octomap::ColorOcTree* colorTree_;
  octomap::OcTree* nonColorTree_;

  sensor_msgs::PointCloud2 odometryPC2;
  sensor_msgs::PointCloud2 mapPC2;

  ros::NodeHandle nh_;

  ros::Publisher transPub;
  ros::Publisher odometryPointPub;
  ros::Publisher colorMapPointCloudPub;
  ros::Publisher nonColorMapPointCloudPub;

  image_transport::ImageTransport it = image_transport::ImageTransport(ros::NodeHandle());
  image_transport::Publisher imgPub;

  pcl::VoxelGrid<pcl::PointXYZRGB> voxelFilter_;
  double voxelSize_;
  PinholeModel pinholeModel_;
};



#endif //DVL_SLAM_MODIFY_LOGGER_H
