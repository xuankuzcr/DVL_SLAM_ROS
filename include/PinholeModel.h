//
// Created by jongsik on 20. 12. 4..
//

#ifndef DVL_SLAM_MODIFY_PINHOLEMODEL_H
#define DVL_SLAM_MODIFY_PINHOLEMODEL_H

#include "Config.h"
#include "Datatypes.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PinholeModel{
public:
  PinholeModel(Config &config);
  ~PinholeModel();

  Eigen::Vector2f PointCloudXyz2Uv(Eigen::Vector3f point);
  std::vector<Eigen::Vector2f> PointCloudXyz2UvVec(const pcl::PointCloud<pcl::PointXYZRGB>& pc, float scale);

private:
  Config &config_;

};

#endif //DVL_SLAM_MODIFY_PINHOLEMODEL_H
