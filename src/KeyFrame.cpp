//
// Created by jongsik on 20. 11. 11..
//
#include "KeyFrame.h"

KeyFrame::KeyFrame(Config &config, Frame::Ptr frame)
  :config_(config),
  frame(frame),
  pinholeModel_(config)
{
}

KeyFrame::~KeyFrame()
{

}

float KeyFrame::GetVisibleRatio (const KeyFrame::Ptr keyframe)
{

  std::cout << "[KeyFrame] Start GetVisibleRatio" << std::endl;

  Sophus::SE3f Tij = frame->GetTwc().inverse() * keyframe->frame->GetTwc();

  int border = config_.trackerConfig.border;
  int currentLevel = 0;

  cv::Mat& current_img = frame->GetPyramidImg(currentLevel);

  const float scale = 1.0f / (1 << currentLevel);

  int visiblePoints = 0;
  int inCameraPoints = 0;

  std::cout << "[KeyFrame] Start2 GetVisibleRatio" << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> currFramePointCloud = keyframe->frame->GetOriginalCloud();

  Eigen::Affine3f transformPitch(Tij.matrix());
  pcl::transformPointCloud(currFramePointCloud, currFramePointCloud, transformPitch);
  std::vector<Eigen::Vector2f> uvSet = pinholeModel_.PointCloudXyz2UvVec(currFramePointCloud, scale);
  auto pointCloudIter = currFramePointCloud.begin();

  for (auto iter=uvSet.begin(); iter!=uvSet.end(); ++iter, ++pointCloudIter) {

    Eigen::Vector2f uv_prev = *iter;

    const float u_prev_f = uv_prev(0);
    const float v_prev_f = uv_prev(1);
    const int u_prev_i = static_cast<int> (u_prev_f);
    const int v_prev_i = static_cast<int> (v_prev_f);

    if (u_prev_i - border < 0 || u_prev_i + border > config_.imageConfig.width || v_prev_i - border < 0 || v_prev_i + border > config_.imageConfig.height || pointCloudIter->z <= 0)
      continue;

    visiblePoints++;

  }
  std::cout << "[KeyFrame] visible_points : " << visiblePoints << std::endl;
  std::cout << "[KeyFrame] keyframe->frame->GetOriginalCloud().size() : " << keyframe->frame->GetOriginalCloud().size() << std::endl;

  return static_cast<float> (visiblePoints) / static_cast<float> (keyframe->frame->GetOriginalCloud().size());
}