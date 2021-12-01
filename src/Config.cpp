//
// Created by jongsik on 20. 10. 28..
//

#include "Config.h"
#include <iostream>

Config::Config()
{

  YAML::Node yamlFile = YAML::LoadFile("/home/chunran/dvl_ws/src/DVL_SLAM_ROS/yaml/defalut_param.yaml");
  ReadEveryParameter(yamlFile);
}


Config::~Config()
{


}


void Config::ReadEveryParameter(const YAML::Node yamlFile)
{
  std::cout<<"111111111111"<<std::endl;

  YAML::Node datasetYaml = yamlFile["Dataset"];
  YAML::Node cameraYaml = yamlFile["Camera"];
  YAML::Node extrinsicYaml = yamlFile["Extrinsic"];
  YAML::Node rectifyingYaml = yamlFile["Rectifying"];
  YAML::Node imageYaml = yamlFile["Image"];
  YAML::Node trackerYaml = yamlFile["Tracker"];
  YAML::Node pointcloudYaml = yamlFile["PointCloud"];
  YAML::Node loggerYaml = yamlFile["Logger"];
  YAML::Node systemYaml = yamlFile["System"];

  std::cout<<"2222222222222"<<std::endl;

  datasetConfig.isKitti = datasetYaml["isKitti"].as<bool>();
  datasetConfig.isIndoor = datasetYaml["isIndoor"].as<bool>();
  datasetConfig.useRos = datasetYaml["useRos"].as<bool>();
  // datasetConfig.imgDir = datasetYaml["imgDir"].as<std::string>();
  // datasetConfig.lidarDir = datasetYaml["lidarDir"].as<std::string>();
  datasetConfig.visualize = datasetYaml["visualize"].as<bool>();

  cameraConfig.fx = cameraYaml["fx"].as<float>();
  cameraConfig.fy = cameraYaml["fy"].as<float>();
  cameraConfig.cx = cameraYaml["cx"].as<float>();
  cameraConfig.cy = cameraYaml["cy"].as<float>();
  cameraConfig.k1 = cameraYaml["k1"].as<float>();
  cameraConfig.k2 = cameraYaml["k2"].as<float>();
  cameraConfig.p1 = cameraYaml["p1"].as<float>();
  cameraConfig.p2 = cameraYaml["p2"].as<float>();
  cameraConfig.k3 = cameraYaml["k3"].as<float>();

  std::cout<<"5555555555555"<<std::endl;

  extrinsicConfig.delX = extrinsicYaml["delX"].as<float>();
  extrinsicConfig.delY = extrinsicYaml["delY"].as<float>();
  extrinsicConfig.delZ = extrinsicYaml["delZ"].as<float>();

  extrinsicConfig.r11 = extrinsicYaml["r11"].as<float>();
  extrinsicConfig.r12 = extrinsicYaml["r12"].as<float>();
  extrinsicConfig.r13 = extrinsicYaml["r13"].as<float>();
  extrinsicConfig.r21 = extrinsicYaml["r21"].as<float>();
  extrinsicConfig.r22 = extrinsicYaml["r22"].as<float>();
  extrinsicConfig.r23 = extrinsicYaml["r23"].as<float>();
  extrinsicConfig.r31 = extrinsicYaml["r31"].as<float>();
  extrinsicConfig.r32 = extrinsicYaml["r32"].as<float>();
  extrinsicConfig.r33 = extrinsicYaml["r33"].as<float>();

  rectifyingConfig.R11 = rectifyingYaml["R11"].as<float>();
  rectifyingConfig.R12 = rectifyingYaml["R12"].as<float>();
  rectifyingConfig.R13 = rectifyingYaml["R13"].as<float>();
  rectifyingConfig.R21 = rectifyingYaml["R21"].as<float>();
  rectifyingConfig.R22 = rectifyingYaml["R22"].as<float>();
  rectifyingConfig.R23 = rectifyingYaml["R23"].as<float>();
  rectifyingConfig.R31 = rectifyingYaml["R31"].as<float>();
  rectifyingConfig.R32 = rectifyingYaml["R32"].as<float>();
  rectifyingConfig.R33 = rectifyingYaml["R33"].as<float>();
  
  std::cout<<"3333333333333333333"<<std::endl;

  imageConfig.width = imageYaml["width"].as<int>();
  imageConfig.height = imageYaml["height"].as<int>();

  trackerConfig.imgPyramidMinLevel = trackerYaml["imgPyramidMinLevel"].as<int>();
  trackerConfig.imgPyramidMaxLevel = trackerYaml["imgPyramidMaxLevel"].as<int>();
  trackerConfig.maxIteration = trackerYaml["maxIteration"].as<int>();
  trackerConfig.normXThresForIteration = trackerYaml["normXThresForIteration"].as<float>();
  trackerConfig.border = trackerYaml["border"].as<int>();

  pointcloudConfig.minZ = pointcloudYaml["minZ"].as<float>();
  pointcloudConfig.maxZ = pointcloudYaml["maxZ"].as<float>();

  loggerConfig.voxelSize = loggerYaml["voxelSize"].as<float>();

  systemConfig.ratioThres = systemYaml["ratioThres"].as<float>();
  std::cout<<"44444444444444444"<<std::endl;

}