//
// Created by jongsik on 20. 12. 18..
//

#include "Logger.h"

Logger::Logger(Config& config)
  : pinholeModel_(config)
  , config_(config)
{
  mapColorPointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  mapNonColorPointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  odometryPointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  imgPub =  it.advertise("/camera/image", 1);
  transPub = nh_.advertise<geometry_msgs::PoseStamped>("/pose_result", 1);
  odometryPointPub = nh_.advertise<sensor_msgs::PointCloud2>("/odometry_result", 1);
  colorMapPointCloudPub = nh_.advertise<sensor_msgs::PointCloud2>("/color_map_result", 1);
  nonColorMapPointCloudPub = nh_.advertise<sensor_msgs::PointCloud2>("/noncolor_octomap_result", 1);

  voxelSize_ = config_.loggerConfig.voxelSize;

  colorTree_ = new octomap::ColorOcTree(voxelSize_);
  nonColorTree_ = new octomap::OcTree(voxelSize_);
}

Logger::~Logger()
{

}

void Logger::PushBackOdometryResult(pcl::PointXYZ odometryPoint)
{
  this->odometryPointCloud_->push_back(odometryPoint);
}

void Logger::PushBackColorMapResult(pcl::PointCloud<pcl::PointXYZRGB> mapCloud, Sophus::SE3f T)
{
  mapColorPointCloud_->clear();
  pcl::PointCloud<pcl::PointXYZRGB> temp;

  int border = config_.trackerConfig.border;

  std::vector<Eigen::Vector2f> uvSet = pinholeModel_.PointCloudXyz2UvVec(mapCloud, 1);
  auto pointCloudIter = mapCloud.begin();
  for(auto uvIter=uvSet.begin(); uvIter!=uvSet.end(); ++uvIter, ++pointCloudIter)
  {
    Eigen::Vector2f uv = *uvIter;

    const float uFloat = uv(0);
    const float vFloat = uv(1);
    const int uInt = static_cast<int> (uFloat);
    const int vInt = static_cast<int> (vFloat);

    if(uInt - border < 0 || uInt + border > config_.imageConfig.width || vInt - border < 0 || vInt + border > config_.imageConfig.height || pointCloudIter->z <= 0.0){
      continue;
    }

    pcl::PointXYZRGB point;
    point.x = pointCloudIter->x;
    point.y = pointCloudIter->y;
    point.z = pointCloudIter->z;

    point.r = pointCloudIter->r;
    point.g = pointCloudIter->g;
    point.b = pointCloudIter->b;

    temp.push_back(point);
  }

  pcl::PointCloud<pcl::PointXYZRGB> transformedTemp;

  pcl::transformPointCloud(temp, transformedTemp, T.matrix());

  for (auto p:transformedTemp.points)
  {
    // The point cloud of points into the octomap
    colorTree_->updateNode( octomap::point3d(p.x, p.y, p.z), true );
  }
  for (auto p:transformedTemp.points)
  {
    colorTree_->integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
  }

  colorTree_->updateInnerOccupancy();

  pcl::PointXYZRGB Point;
  for (octomap::ColorOcTree::leaf_iterator it = colorTree_->begin_leafs(), end = colorTree_->end_leafs(); it != end; ++it) {
    if (colorTree_->isNodeOccupied(*it)) {
      Point.x = it.getX();
      Point.y = it.getY();
      Point.z = it.getZ();
      Point.r = it->getColor().r;
      Point.g = it->getColor().g;
      Point.b = it->getColor().b;

      mapColorPointCloud_->push_back(Point);
    }
  }
}

void Logger::PushBackNonColorMapResult(pcl::PointCloud<pcl::PointXYZRGB> mapCloud, Sophus::SE3f T)
{
  mapNonColorPointCloud_->clear();
  pcl::PointCloud<pcl::PointXYZRGB> temp;
  pcl::transformPointCloud(mapCloud, temp, T.matrix());

  for (auto p:temp.points)
  {
    nonColorTree_->updateNode( octomap::point3d(p.x, p.y, p.z), true );
  }

  nonColorTree_->updateInnerOccupancy();

  pcl::PointXYZ Point;

  for (octomap::OcTree::leaf_iterator it = nonColorTree_->begin_leafs(), end = nonColorTree_->end_leafs(); it != end; ++it) {
    if (nonColorTree_->isNodeOccupied(*it)) {
      Point.x = it.getX();
      Point.y = it.getY();
      Point.z = it.getZ();
      mapNonColorPointCloud_->push_back(Point);
    }
  }
}

void Logger::SaveMapResult()
{

}

void Logger::SaveOdometryResult()
{

}

void Logger::PublishImg(cv::Mat image){
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(header, "bgr8", image);
  img_bridge.toImageMsg(img_msg);
  imgPub.publish(img_msg);
}

void Logger::PublishTransform(Sophus::SE3f input){

  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = input.translation().x();
  msg.pose.position.y = input.translation().y();
  msg.pose.position.z = input.translation().z();
  msg.pose.orientation.x = input.unit_quaternion().x();
  msg.pose.orientation.y = input.unit_quaternion().y();
  msg.pose.orientation.z = input.unit_quaternion().z();
  msg.pose.orientation.w = input.unit_quaternion().w();
  msg.header.frame_id = "world";

//  geometry_msgs::Transform msg;
//  msg.translation.x = input.translation().x();
//  msg.translation.y = input.translation().y();
//  msg.translation.y = input.translation().y();
//  msg.rotation.x = input.unit_quaternion().x();
//  msg.rotation.x = input.unit_quaternion().y();
//  msg.rotation.x = input.unit_quaternion().z();
//  msg.rotation.x = input.unit_quaternion().w();

  transPub.publish(msg);
}

void Logger::PublishOdometryPoint(){
  pcl::toROSMsg(*odometryPointCloud_, odometryPC2);
  odometryPC2.header.frame_id = "world";
  odometryPC2.header.stamp = ros::Time::now();
  odometryPointPub.publish(odometryPC2);
}

void Logger::PublishNonColorMapPointCloud(){
  pcl::toROSMsg(*mapNonColorPointCloud_, mapPC2);
  mapPC2.header.frame_id = "world";
  mapPC2.header.stamp = ros::Time::now();
  nonColorMapPointCloudPub.publish(mapPC2);
}

void Logger::PublishColorMapPointCloud(){
  pcl::toROSMsg(*mapColorPointCloud_, mapPC2);
  mapPC2.header.frame_id = "world";
  mapPC2.header.stamp = ros::Time::now();
  colorMapPointCloudPub.publish(mapPC2);
}

