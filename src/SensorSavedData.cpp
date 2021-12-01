//
// Created by jongsik on 20. 12. 11..
//

#include "SensorSavedData.h"


SensorSavedData::SensorSavedData(Config &config)
: Sensor(config)
, config_(config)
{
  std::cout << "[SensorSavedData] file loading.." << std::endl;
  loadPointCloud();
  loadImg();
  lidarFlag_ = true;
  imgFlag_ = true;
  std::cout << "[SensorSavedData] file load finished!" << std::endl;
}

SensorSavedData::~SensorSavedData()
{
}

void SensorSavedData::data2Frame(Frame& frame)
{
  inputImg_ = imgVec.front();
  inputCloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(cloudVec.front()));
  imgVec.pop();
  cloudVec.pop();

  frame.SetOriginalImg(inputImg_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rectifiedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  transform(0, 0) = config_.extrinsicConfig.r11;
  transform(0, 1) = config_.extrinsicConfig.r12;
  transform(0, 2) = config_.extrinsicConfig.r13;

  transform(1, 0) = config_.extrinsicConfig.r21;
  transform(1, 1) = config_.extrinsicConfig.r22;
  transform(1, 2) = config_.extrinsicConfig.r23;

  transform(2, 0) = config_.extrinsicConfig.r31;
  transform(2, 1) = config_.extrinsicConfig.r32;
  transform(2, 2) = config_.extrinsicConfig.r33;

  transform(0, 3) = config_.extrinsicConfig.delX;
  transform(1, 3) = config_.extrinsicConfig.delY;
  transform(2, 3) = config_.extrinsicConfig.delZ;

  pcl::transformPointCloud(*inputCloud_, *transformedCloud, transform);

  Eigen::Matrix4f transformRect = Eigen::Matrix4f::Identity();

  transformRect(0, 0) = config_.rectifyingConfig.R11;
  transformRect(0, 1) = config_.rectifyingConfig.R12;
  transformRect(0, 2) = config_.rectifyingConfig.R13;

  transformRect(1, 0) = config_.rectifyingConfig.R21;
  transformRect(1, 1) = config_.rectifyingConfig.R22;
  transformRect(1, 2) = config_.rectifyingConfig.R23;

  transformRect(2, 0) = config_.rectifyingConfig.R31;
  transformRect(2, 1) = config_.rectifyingConfig.R32;
  transformRect(2, 2) = config_.rectifyingConfig.R33;

  transformRect(0, 3) = 0;
  transformRect(1, 3) = 0;
  transformRect(2, 3) = 0;

  pcl::transformPointCloud(*transformedCloud, *rectifiedCloud, transformRect);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(rectifiedCloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(minZ, maxZ);
  pass.filter(*rectifiedCloud);

  frame.SetOriginalCloud(*rectifiedCloud);
}

void SensorSavedData::loadImg()
{
  std::vector<cv::String> fn;
  std::string filename = config_.datasetConfig.imgDir + "*.png";
  glob(filename.c_str(), fn, false);

  size_t count = fn.size();
  for(size_t i=0; i<count; i++){
    imgVec.push(cv::imread(fn[i]));
  }
}

void SensorSavedData::loadPointCloud()
{
  boost::filesystem::path lidarDir(config_.datasetConfig.lidarDir);

  std::vector<boost::filesystem::path> filenameVec;
  copy(boost::filesystem::directory_iterator(lidarDir), boost::filesystem::directory_iterator(), std::back_inserter(filenameVec));
  std::sort(filenameVec.begin(), filenameVec.end());

  for(auto& boostFilename:filenameVec){

    pcl::PointCloud<pcl::PointXYZRGB> occupiedCloud;
    pcl::PointXYZRGB occupiedPoint;
    std::string str_buf;
    float f_buf;
    std::fstream fs;
    std::string filename = boostFilename.string();
    fs.open(filename.c_str(), std::ios::in);
    int i=0;
    while (!fs.eof()){
      getline(fs, str_buf, ',');
      if(str_buf.size() > 2){
        f_buf = stof(str_buf);
        if (i%4 == 0){
          occupiedPoint.x = f_buf;
        }
        else if(i%4 == 1){
          occupiedPoint.y = f_buf;
        }
        else if(i%4 == 2){
          occupiedPoint.z = f_buf;
          occupiedCloud.points.push_back(occupiedPoint);
        }
        i++;
      }
    }
    cloudVec.push(occupiedCloud);
  }
}