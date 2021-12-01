//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_TRACKER_H
#define DVL_SLAM_MODIFY_TRACKER_H

#include "Config.h"
#include "Frame.h"
#include "KeyFrame.h"
#include <sophus/se3.hpp>
#include "Datatypes.h"
#include "PinholeModel.h"

class Tracker{

  static const int patchHalfsize_ = 2;
  static const int patchSize_ = 2*patchHalfsize_;
  static const int patternLength_ = 8;
  int pattern_[8][2] = { {0, 0}, {2, 0}, {1, 1}, {0, -2}, {-1, -1}, {-2, 0}, {-1, 1}, {0, 2} };

public:

  Tracker(Config &config);
  ~Tracker();

  Config& config_;

  bool Solve();
  void UpdatePose(const Sophus::SE3f& old_Tji, Sophus::SE3f& Tji);
  void Optimize(Sophus::SE3f& Tji);
  float HuberWeight(const float res);
  void CheckVisiblePointsInPrevFrame(Frame::Ptr currFrame, Sophus::SE3f& transformation);
  cv::Mat PrecomputePatches(cv::Mat& img, pcl::PointCloud<pcl::PointXYZRGB>& pointcloud, cv::Mat& patch_buf, bool is_derivative);
  double ComputeResiduals(Sophus::SE3f& transformation);
  void trackFrame2Frame(Frame::Ptr currFrame, KeyFrame::Ptr refFrame, Sophus::SE3f& transformation);

  inline double NormMax(const Vector6& v)
  {
    double max = -1;
    for (int i=0; i<v.size(); i++)
    {
      double abs = fabs(v[i]);
      if(abs>max){
        max = abs;
      }
    }
    return max;
  }

  inline float compute(cv::Mat& errors)
  {
    float initial_sigma_ = 5.0f;
    float dof_ = 5.0f;
    float initial_lamda = 1.0f / (initial_sigma_ * initial_sigma_);
    int num = 0;
    float lambda = initial_lamda;
    int iterations = 0;
    do
    {
      ++iterations;
      initial_lamda = lambda;
      num = 0.0f;
      lambda = 0.0f;

      const float* data_ptr = errors.ptr<float>();

      for(size_t idx = 0; idx < errors.size().area(); ++idx, ++data_ptr)
      {
        const float& data = *data_ptr;

        if(std::isfinite(data))
        {
          ++num;
          lambda += data * data * ( (dof_ + 1.0f) / (dof_ + initial_lamda * data * data) );
        }
      }

      lambda = float(num) / lambda;
    } while(std::abs(lambda - initial_lamda) > 1e-3);

    return std::sqrt(1.0f / lambda);
  }

  inline float calcWeight(const float &res)
  {
    float dof_ = 5.0f;
    return (dof_ + 1.0) / (dof_ + res*res);
  }
private:
  Frame::Ptr currentFrame_;
  KeyFrame::Ptr referenceFrame_;
  PinholeModel pinholeModel_;

  float huberK_;

  Vector6 x_;
  std::vector<Vector6> J_;
  Vector6 Jres_;
  Matrix6x6 H_;
  Matrix6x6 prev_H_;

  std::vector<float> weight_;
  cv::Mat refPatchBuf_;
  cv::Mat currPatchBuf_;

  std::vector<float> errors_;

  int minLevel_;
  int maxLevel_;
  int currentLevel_;

  Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> dIBuf_;
  Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> jacobianBuf_;

  float scale_;

  int maxIteration;
  float residual_;
  float normXThres_;
  bool status_;


  float affine_a_ = 1;
  float affine_b_ = 0;
  bool stop_;
  bool isPreComputed_;

  cv::Mat refImgClone;
  cv::Mat currImgClone;
};


#endif //DVL_SLAM_MODIFY_TRACKER_H
