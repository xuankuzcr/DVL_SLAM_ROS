//
// Created by jongsik on 20. 12. 11..
//

#include "SensorRos.h"

SensorRos::SensorRos(Config &config)
: Sensor(config)
{
  std::string imgTopic;
  std::string pcTopic;
  if(config_.datasetConfig.isIndoor)
  {
    imgTopic = "/left_camera/image";
    pcTopic = "/livox_pcl0";
  }
  else if(config_.datasetConfig.isKitti)
  {
    imgTopic = "/kitti/camera_color_left/image_raw";
    pcTopic = "/kitti/velo/pointcloud";
  }

  imgSub = nh_.subscribe(imgTopic, 1, &SensorRos::ImgCb, this);
  pointCloudSub = nh_.subscribe(pcTopic, 1, &SensorRos::PcCb, this);
}

SensorRos::~SensorRos()
{
}

void SensorRos::ImgCb(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
  inputImg_ = cv_ptr->image.clone();
  imgFlag_ = true;
}

void SensorRos::PcCb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *inputCloud_);
  lidarFlag_ = true;
}

void SensorRos::data2Frame(Frame& frame){
  frame.SetOriginalImg(inputImg_);
  if(lidarFlag_) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectifiedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);


    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    transform (0,0) = config_.extrinsicConfig.r11;
    transform (0,1) = config_.extrinsicConfig.r12;
    transform (0,2) = config_.extrinsicConfig.r13;

    transform (1,0) = config_.extrinsicConfig.r21;
    transform (1,1) = config_.extrinsicConfig.r22;
    transform (1,2) = config_.extrinsicConfig.r23;

    transform (2,0) = config_.extrinsicConfig.r31;
    transform (2,1) = config_.extrinsicConfig.r32;
    transform (2,2) = config_.extrinsicConfig.r33;

    transform (0,3) = config_.extrinsicConfig.delX;
    transform (1,3) = config_.extrinsicConfig.delY;
    transform (2,3) = config_.extrinsicConfig.delZ;

    pcl::transformPointCloud (*inputCloud_, *transformedCloud, transform);

    Eigen::Matrix4f transformRect = Eigen::Matrix4f::Identity();

    transformRect (0,0) = config_.rectifyingConfig.R11;
    transformRect (0,1) = config_.rectifyingConfig.R12;
    transformRect (0,2) = config_.rectifyingConfig.R13;

    transformRect (1,0) = config_.rectifyingConfig.R21;
    transformRect (1,1) = config_.rectifyingConfig.R22;
    transformRect (1,2) = config_.rectifyingConfig.R23;

    transformRect (2,0) = config_.rectifyingConfig.R31;
    transformRect (2,1) = config_.rectifyingConfig.R32;
    transformRect (2,2) = config_.rectifyingConfig.R33;

    transformRect (0,3) = 0;
    transformRect (1,3) = 0;
    transformRect (2,3) = 0;

    pcl::transformPointCloud (*transformedCloud, *rectifiedCloud, transformRect);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(rectifiedCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(minZ, maxZ);
    pass.filter(*rectifiedCloud);


    frame.SetOriginalCloud(*rectifiedCloud);
  }

}