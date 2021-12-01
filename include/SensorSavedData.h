//
// Created by jongsik on 20. 12. 11..
//

#ifndef DVL_SLAM_MODIFY_SENSORSAVEDDATA_H
#define DVL_SLAM_MODIFY_SENSORSAVEDDATA_H

#include "Sensor.h"
#include "glob.h"
#include "boost/filesystem.hpp"

class SensorSavedData : public Sensor{
public:
  SensorSavedData(Config& config);
  ~SensorSavedData();

  void loadImg();
  void loadPointCloud();

  void data2Frame(Frame& frame) override;

private:
  Config& config_;
  std::queue<cv::Mat> imgVec;
  std::queue<pcl::PointCloud<pcl::PointXYZRGB>> cloudVec;
};

#endif //DVL_SLAM_MODIFY_SENSORSAVEDDATA_H
