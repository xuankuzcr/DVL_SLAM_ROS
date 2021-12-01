//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_FRAME_H
#define DVL_SLAM_MODIFY_FRAME_H
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include "Config.h"
#include "Datatypes.h"
#include "sophus/se3.hpp"
#include <mutex>
#include "PinholeModel.h"

void createImagePyramid(const cv::Mat& img_level_0, int n_levels, ImgPyramid& pyr);

class Frame {
public:
  typedef std::shared_ptr<Frame> Ptr;

  Frame(Config &config);
  ~Frame();

  cv::Mat PointCloud2Img();

  void setImg();
  void setPointCloud();

  void showImg(cv::Mat& img);
  void saveImg(cv::Mat& img);

  cv::Mat&  GetPyramidImg(size_t level);
  void SetOriginalImg(cv::Mat originalImg);
  cv::Mat GetOriginalImg();

  pcl::PointCloud<pcl::PointXYZRGB> GetOriginalCloud();
  void SetOriginalCloud(pcl::PointCloud<pcl::PointXYZRGB>& originalCloud);

  Sophus::SE3f GetTwc();
  void SetTwc(Sophus::SE3f Twc);

  Eigen::Vector2f PointCloudXyz2Uv(Eigen::Vector3f point);
  std::vector<Eigen::Vector2f> PointCloudXyz2UvVec(const pcl::PointCloud<pcl::PointXYZRGB>& pc);

  cv::Mat pointCloudProjection();

  void createImagePyramid();

  template<typename T>
  void pyrDownMeanSmooth(const cv::Mat& in, cv::Mat& out);

  inline static void jacobian_xyz2uv(const Eigen::Vector3f& xyzFloat, Matrix2x6& J)
  {
    const float x = xyzFloat[0];
    const float y = xyzFloat[1];
    const float zInv = 1./xyzFloat[2];
    const float zInv2 = zInv*zInv;

    J(0,0) = -zInv;              // -1/z
    J(0,1) = 0.0;                 // 0
    J(0,2) = x*zInv2;           // x/z^2
    J(0,3) = y*J(0,2);            // x*y/z^2
    J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
    J(0,5) = y*zInv;             // y/z


    J(1,0) = 0.0;                 // 0
    J(1,1) = -zInv;              // -1/z
    J(1,2) = y*zInv2;           // y/z^2
    J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
    J(1,4) = -J(0,3);             // -x*y/z^2
    J(1,5) = -x*zInv;            // x/z
  }
private:
  Config &config_;

  cv::Mat originalImg_;
  cv::Mat gray_;
  pcl::PointCloud<pcl::PointXYZRGB> originalCloud_;

  Sophus::SE3f Twc_;

  ImgPyramid imgPyramid_;
  PinholeModel pinholeModel_;
  int numLevel_;
};


class FrameDB
{
public:
  typedef std::shared_ptr<FrameDB> Ptr;

  FrameDB() {}
  ~FrameDB() {}

  void Add(Frame::Ptr frame) {
    boost::unique_lock<std::mutex> ul{DB_mutex_};
    frameDB_.push_back(frame);
    ul.unlock();
  }

  std::vector<Frame::Ptr>::iterator begin() { return frameDB_.begin(); }
  std::vector<Frame::Ptr>::iterator end() { return frameDB_.end(); }
  size_t size() { return frameDB_.size(); }
  std::vector<Frame::Ptr>& frameDB() { return frameDB_; }
private:
  std::vector<Frame::Ptr> frameDB_;
  std::mutex DB_mutex_;
};

#endif //DVL_SLAM_MODIFY_FRAME_H
