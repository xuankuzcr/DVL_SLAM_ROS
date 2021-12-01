//
// Created by jongsik on 20. 10. 30..
//

#include "System.h"

System::System(Config& config)
  : config_(config),
    graphOptimizer_(config),
    tracker_(config),
    logger_(config),
    Tij_(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f)),
    Tji_(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f)),
    dTji_(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f))
{
  initialized_ = false;
  num = 0;

  keyFrameDB_.reset(new KeyFrameDB());
  frameDB_.reset(new FrameDB());
  if(config_.datasetConfig.useRos) { sensor_ = new SensorRos(config); }
  else { sensor_ = new SensorSavedData(config); }
}

System::~System()
{

}

void System::Run(){
  Frame::Ptr currFrame (new Frame(config_));
  if(!sensor_->IsLidarSubscribed() || !sensor_->IsVisionSubscribed()){
    std::cout << "[System] No sensor data subscribed" << std::endl;
    return;
  }
  sensor_->data2Frame(*currFrame);

  std::cout << "[System] frame class got data from sensor class" << std::endl;

  if(!initialized_){
    Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
    Eigen::Vector3f twc(0, 0, 0);

    Sophus::SE3f initialTwc(rot, twc);
    currFrame->SetTwc(initialTwc);

    KeyFrame::Ptr keyFrame(new KeyFrame(config_, currFrame));
    frameDB_->Add(currFrame);
    keyFrameDB_->Add(keyFrame);

    initialized_ = true;

    pcl::PointXYZ odometryLogger;
    odometryLogger.x = initialTwc.translation()[0];
    odometryLogger.y = initialTwc.translation()[1];
    odometryLogger.z = initialTwc.translation()[2];

    logger_.PushBackOdometryResult(odometryLogger);
    logger_.PushBackColorMapResult(currFrame->GetOriginalCloud(), initialTwc);
    logger_.PushBackNonColorMapResult(currFrame->GetOriginalCloud(), initialTwc);
    logger_.PublishOdometryPoint();
    logger_.PublishNonColorMapPointCloud();
    logger_.PublishColorMapPointCloud();

    std::cout << "[System] Initialized" << std::endl;
    return;
  }
  else{

    std::cout << "[System] Find KeyFrame from KeyFrame DB" << std::endl;

    KeyFrame::Ptr lastKeyFrame = keyFrameDB_->LatestKeyframe();

    Sophus::SE3f prevTji = Tji_;
    Tji_ = Tji_ * dTji_;

    tracker_.trackFrame2Frame(currFrame, lastKeyFrame, Tji_);
    dTji_ = Tji_ * prevTji.inverse();
    Tij_ = Tji_.inverse();
    std::cout << "[System] Tracking Finished" << std::endl;

    Sophus::SE3f Twc = lastKeyFrame->frame->GetTwc();
    Sophus::SE3f T = Twc * Tij_;

    std::ofstream foutC("/home/chunran/CameraTrajectory.txt", std::ios::app|std::ios::out);
    foutC.setf(std::ios::fixed, std::ios::floatfield);
    foutC.precision(9); //9 timestamp seconds
    foutC << num << " ";
    num = num + 0.1;
    foutC.precision(5);

    Eigen::Matrix<float, 3, 3> R = T.rotationMatrix();
    Eigen::Vector3f t = T.translation();

    Eigen::Quaternionf q(R);
    q.normalized();

    float x = t(0);
    float y = t(1);
    float z = t(2);
    float qx = q.x();
    float qy = q.y();
    float qz = q.z();
    float qw = q.w();

    foutC << x <<" " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;

    std::cout << "[System] T = " << std::endl;
    std::cout << T.matrix() << std::endl;

    currFrame->SetTwc(Twc * Tij_);

    float ratio_threshold = config_.systemConfig.ratioThres;
    std::cout << "[System] Find Keyframe" << std::endl;

    KeyFrame::Ptr currentKeyframe(new KeyFrame(config_, currFrame));

    float visible_ratio1 = lastKeyFrame->GetVisibleRatio(currentKeyframe);
    float visible_ratio2 = currentKeyframe->GetVisibleRatio(lastKeyFrame);

    bool is_keyframe = (visible_ratio1 < ratio_threshold ? true : false) || ((visible_ratio2 < ratio_threshold ? true : false));

    std::cout << "[System] KeyFrame Decision calculated" << std::endl;

    if(is_keyframe)
    {
      keyFrameDB_->Add(currentKeyframe);
      pcl::PointXYZ odometryLogger;
      odometryLogger.x = T.translation()[0];
      odometryLogger.y = T.translation()[1];
      odometryLogger.z = T.translation()[2];


      logger_.PushBackOdometryResult(odometryLogger);
      logger_.PushBackColorMapResult(currFrame->GetOriginalCloud(), T);
      logger_.PushBackNonColorMapResult(currFrame->GetOriginalCloud(), T);
      logger_.PublishOdometryPoint();
      logger_.PublishNonColorMapPointCloud();
      logger_.PublishColorMapPointCloud();

      Tji_.rotationMatrix() = Eigen::Matrix3f::Identity();
      Tji_.translation() = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

      std::cout << "[System] Add KeyFrame" << std::endl;

    }

    std::cout << "[System] Finished" << std::endl;

    // sensor_->publishTransform(Twc * Tij_);
  }

}