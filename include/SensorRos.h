//
// Created by jongsik on 20. 12. 11..
//

#ifndef DVL_SLAM_MODIFY_SENSORROS_H
#define DVL_SLAM_MODIFY_SENSORROS_H

#include "Sensor.h"

class SensorRos : public Sensor{
public:
  SensorRos(Config& config);
  ~SensorRos();
  void ImgCb(const sensor_msgs::ImageConstPtr& img);
  void PcCb(const sensor_msgs::PointCloud2ConstPtr& pc);

  void data2Frame(Frame& frame) override;
private:
  ros::NodeHandle nh_;
  ros::Subscriber imgSub;
  ros::Subscriber pointCloudSub;
};

#endif //DVL_SLAM_MODIFY_SENSORROS_H
