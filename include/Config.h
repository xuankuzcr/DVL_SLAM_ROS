//
// Created by jongsik on 20. 10. 28..
//

#ifndef DVL_SLAM_MODIFY_CONFIG_H
#define DVL_SLAM_MODIFY_CONFIG_H

#include <yaml-cpp/yaml.h>
struct DatasetConfig
{
  bool isKitti;
  bool isIndoor;
  bool useRos;
  std::string imgDir;
  std::string lidarDir;
  bool visualize;
};

struct CameraConfig
{
  float fx;
  float fy;
  float cx;
  float cy;
  float k1;
  float k2;
  float p1;
  float p2;
  float k3;
};

struct ExtrinsicConfig
{
  float delX;
  float delY;
  float delZ;

  float r11;
  float r12;
  float r13;
  float r21;
  float r22;
  float r23;
  float r31;
  float r32;
  float r33;
};

struct RectifyingConfig
{
  float R11;
  float R12;
  float R13;
  float R21;
  float R22;
  float R23;
  float R31;
  float R32;
  float R33;
};

struct ImageConfig
{
  int width;
  int height;
};

struct TrackerConfig
{
  int imgPyramidMinLevel;
  int imgPyramidMaxLevel;
  int maxIteration;
  float normXThresForIteration;
  int border;
};

struct PointCloudConfig
{
  float minZ;
  float maxZ;
};

struct LoggerConfig
{
  float voxelSize;
};

struct SystemConfig
{
  float ratioThres;
};
class Config{
public:
  Config();
  ~Config();

  DatasetConfig datasetConfig;
  CameraConfig cameraConfig;
  ExtrinsicConfig extrinsicConfig;
  RectifyingConfig rectifyingConfig;
  ImageConfig imageConfig;
  TrackerConfig trackerConfig;
  PointCloudConfig pointcloudConfig;
  LoggerConfig loggerConfig;
  SystemConfig systemConfig;

private:
  void ReadEveryParameter(YAML::Node yamlFile);
};

#endif //DVL_SLAM_MODIFY_CONFIG_H
