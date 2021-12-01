//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_SYSTEM_H
#define DVL_SLAM_MODIFY_SYSTEM_H

#include "Config.h"
#include "Frame.h"
#include "GraphOptimizer.h"
#include "KeyFrame.h"
#include "Sensor.h"
#include "SensorRos.h"
#include "SensorSavedData.h"
#include "Tracker.h"
#include <sophus/se3.hpp>
#include "Logger.h"
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

class System{
public:
  System(Config& config_);
  ~System();

  void Run();

private:
  Config& config_;
  GraphOptimizer graphOptimizer_;
  Sensor* sensor_;
  Tracker tracker_;
  Logger logger_;

  bool initialized_;
  float num;
  Sophus::SE3f Tij_;
  Sophus::SE3f Tji_;
  Sophus::SE3f dTji_;

  FrameDB::Ptr frameDB_;
  KeyFrameDB::Ptr keyFrameDB_;
};


#endif //DVL_SLAM_MODIFY_SYSTEM_H
