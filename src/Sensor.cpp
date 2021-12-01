//
// Created by jongsik on 20. 10. 30..
//

#include "Sensor.h"

Sensor::Sensor(Config& config)
: config_(config)
{
  lidarFlag_ = false;
  imgFlag_ = false;

  minZ = config_.pointcloudConfig.minZ;
  maxZ = config_.pointcloudConfig.maxZ;

  inputCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

Sensor::~Sensor(){
}

bool Sensor::IsLidarSubscribed(){ return lidarFlag_; }

bool Sensor::IsVisionSubscribed(){ return imgFlag_; }