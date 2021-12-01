//
// Created by jongsik on 20. 10. 30..
//

#include <Frame.h>

Frame::Frame(Config &config)
  : config_(config)
  , pinholeModel_(config)
{
  numLevel_ = config_.trackerConfig.imgPyramidMaxLevel - config.trackerConfig.imgPyramidMinLevel + 1;
}

Frame::~Frame(){
  imgPyramid_.clear();
}
template<typename T>
void Frame::pyrDownMeanSmooth(const cv::Mat& in, cv::Mat& out)
{
  for(int y = 0; y < out.rows; ++y)
  {
    for(int x = 0; x < out.cols; ++x)
    {
      int x0 = x * 2;
      int x1 = x0 + 1;
      int y0 = y * 2;
      int y1 = y0 + 1;
      out.at<T>(y, x) = (T) ( (in.at<T>(y0, x0) + in.at<T>(y0, x1) + in.at<T>(y1, x0) + in.at<T>(y1, x1)) / 4.0f );
    }
  }
}

void Frame::createImagePyramid()
{
  imgPyramid_.resize(numLevel_);
  imgPyramid_[0] = gray_;

  for(int i=1; i<numLevel_; ++i)
  {
    imgPyramid_[i] = cv::Mat(imgPyramid_[i-1].rows/2, imgPyramid_[i-1].cols/2, CV_32FC1);
    pyrDownMeanSmooth<float>(imgPyramid_[i-1], imgPyramid_[i]);
  }
}

cv::Mat& Frame::GetPyramidImg(size_t level){ return imgPyramid_[level]; }

cv::Mat Frame::GetOriginalImg(){ return originalImg_; }

void Frame::SetOriginalImg(cv::Mat originalImg){
  this->originalImg_ = originalImg;
  originalImg_.convertTo(originalImg_, CV_32FC3, 1.0/255);
  cvtColor(originalImg, gray_, cv::COLOR_BGR2GRAY);
  gray_.convertTo(gray_, CV_32FC1, 1.0/255);

  createImagePyramid();
}

pcl::PointCloud<pcl::PointXYZRGB> Frame::GetOriginalCloud(){ return originalCloud_; }

void Frame::SetOriginalCloud(pcl::PointCloud<pcl::PointXYZRGB>& originalCloud)
{
  originalCloud_.clear();
  int border = config_.trackerConfig.border;
  std::vector<Eigen::Vector2f> uvSet = pinholeModel_.PointCloudXyz2UvVec(originalCloud, 1);
  auto pointCloudIter = originalCloud.begin();
  pcl::PointXYZRGB temp;
  for(auto uvIter=uvSet.begin(); uvIter!=uvSet.end(); ++uvIter, ++pointCloudIter){
    Eigen::Vector2f uv = *uvIter;

    const float uFloat = uv(0);
    const float vFloat = uv(1);
    const int uInt = static_cast<int> (uFloat);
    const int vInt = static_cast<int> (vFloat);

    if(uInt - border < 0 || uInt + border > config_.imageConfig.width || vInt - border < 0 || vInt + border > config_.imageConfig.height || pointCloudIter->z <= 0.0){
      continue;
    }
    temp.b = 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(0);
    temp.g = 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(1);
    temp.r = 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(2);
    temp.x = pointCloudIter->x;
    temp.y = pointCloudIter->y;
    temp.z = pointCloudIter->z;

    originalCloud_.push_back(temp);
//    std::cout << "pointCloudIter->b = 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(0)" << 255*originalImg_.at<cv::Vec3f>(vInt, uInt) << std::endl;
//    std::cout << "pointCloudIter->b = 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(0)" << 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(0) << std::endl;
//    std::cout << "pointCloudIter->b = 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(1)" << 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(1) << std::endl;
//    std::cout << "pointCloudIter->b = 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(2)" << 255*originalImg_.at<cv::Vec3f>(vInt, uInt)(2) << std::endl;

  }

}

Sophus::SE3f Frame::GetTwc(){ return Twc_; }

void Frame::SetTwc(Sophus::SE3f Twc){ this->Twc_ = Twc; }

cv::Mat Frame::pointCloudProjection()
{

  cv::Mat projectedImg = originalImg_.clone();

  for(int i=0; i<originalCloud_.points.size(); i++)
  {
    float U = config_.cameraConfig.fx * (originalCloud_.points[i].x / originalCloud_.points[i].z) + config_.cameraConfig.cx;
    float V = config_.cameraConfig.fy * (originalCloud_.points[i].y / originalCloud_.points[i].z) + config_.cameraConfig.cy;

    float v_min = config_.pointcloudConfig.minZ;    float v_max = config_.pointcloudConfig.maxZ;    float dv = v_max - v_min;
    float v = originalCloud_.points[i].z;
    float r = 1.0; float g = 1.0; float b = 1.0;
    if (v < v_min)   v = v_min;
    if (v > v_max)   v = v_max;

    if(v < v_min + 0.25*dv) {
      r = 0.0;
      g = 4*(v - v_min) / dv;
    }
    else if (v < (v_min + 0.5 * dv)) {
      r = 0.0;
      b = 1 + 4*(v_min + 0.25 * dv - v) / dv;
    }
    else if (v < (v_min + 0.75 * dv)) {
      r =4 * (v - v_min - 0.5 * dv) / dv;
      b = 0.0;
    }
    else {
      g = 1 + 4*(v_min + 0.75 * dv - v) / dv;
      b = 0.0;
    }

    cv::circle(projectedImg, cv::Point(U, V), 2, cv::Scalar(255*r, 255*g, 255*b), 1);
  }
  return projectedImg;
}

