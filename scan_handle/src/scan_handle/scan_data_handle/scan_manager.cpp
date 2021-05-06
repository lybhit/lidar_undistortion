#include "scan_handle/scan_data_handle/scan_manager.h"

ScanManager::ScanManager():lidar_motion_calibrator_("laser_link", "base_link", "odom"){
  
}

ScanManager::~ScanManager()
{
}

void ScanManager::setOptions(Config& config)
{
  laser_min_range_ = config.laser_min_range;
  laser_max_range_ = config.laser_max_range;
  scan_time_from_start_ = config.scan_time_from_start;
  base_frame_id_ = config.base_frame_id;


  std::cout << "laser min range = " <<  laser_min_range_ << std::endl;
  std::cout << "laser max range = " <<  laser_max_range_ << std::endl;
  std::cout << "scan time from start = " <<  scan_time_from_start_ << std::endl;
  std::cout << "base_frame_id = " << base_frame_id_ << std::endl;
}

/*
  filter scan msg by distance and put dato to ranges and angles container. 
*/
void  ScanManager::scanDataCollate(const sensor_msgs::LaserScan::ConstPtr&laser_scan, const double& angle_min, const double& angle_increment)
{
  unsigned int num = laser_scan->ranges.size();

  ranges_.clear();
  angles_.clear();

  ranges_.reserve(num);
  angles_.reserve(num);

  for (int i = 0; i < num; i++) {
    double r, theta;
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if (laser_scan->ranges[i] <= laser_min_range_)
    {
      r = laser_max_range_;
    }else{
      r = laser_scan->ranges[i];
    }

    // Compute bearing
    theta = angle_min + (i * angle_increment);

    ranges_.emplace_back(r);
    angles_.emplace_back(theta);
  }
}

void ScanManager::scanDataTimeHandle(const sensor_msgs::LaserScan::ConstPtr& laser_scan)
{
  int num = laser_scan->ranges.size();
  increment_time_ = laser_scan->time_increment;
  ROS_INFO("laser number = %d, increment_time = %f", num, increment_time_);

  if(scan_time_from_start_)
  {
    start_time_ = laser_scan->header.stamp;
    if(increment_time_ > 0)
    {
      end_time_ = laser_scan->header.stamp + ros::Duration(laser_scan->time_increment * num);
    }else
    {
      end_time_ = laser_scan->header.stamp + ros::Duration(laser_scan->scan_time);
    }
    
  }else
  {
    end_time_ = laser_scan->header.stamp;

    if(increment_time_ > 0)
    {
      start_time_ = laser_scan->header.stamp - ros::Duration(laser_scan->time_increment * num);
    }else
    {
      start_time_ = laser_scan->header.stamp - ros::Duration(laser_scan->scan_time);
    }

  }
  ROS_INFO("end_time = %f, start_time = %f", end_time_.toSec(), start_time_.toSec());
}

void ScanManager::scanDataUndistortion()
{
    lidar_motion_calibrator_.lidarCalibration(ranges_, angles_, start_time_, end_time_, increment_time_);
}

/*
  convert polar coordinate to xyz coordinate.
*/
std::vector<Eigen::Vector3f> ScanManager::scanDataToCoordinate()
{
  std::vector<Eigen::Vector3f> laser_points;

  assert(ranges_.size() == angles_.size());
  int num = ranges_.size();
  for (int i = 0; i < num; i++) {
    Eigen::Vector3f tmp_point;
    tmp_point.x() = ranges_[i] * cos(angles_[i]);
    tmp_point.y() = ranges_[i] * sin(angles_[i]);
    tmp_point.z() = 0.0;

    laser_points.emplace_back(tmp_point);

    if(i % 10 == 0 || i == num - 1)
    {
      ROS_INFO("Before undistortion: index = %d, x = %f, y = %f", i, tmp_point.x(), tmp_point.y());
    }
  }


  //ROS_INFO("laser points size = %d", laser_points.size());

  return laser_points;
}

void ScanManager::calculateLaserToBaseOffset(const std::shared_ptr<tf2_ros::Buffer>& tf)
{
  ROS_INFO("calculate laser to base tf.");
  auto laser_in_base = tf->lookupTransform(base_frame_id_, "laser_link", ros::Time(0));
  
  tf2::convert(laser_in_base.transform, laser_to_base_tf_);

  // ROS_INFO("laser_to_base_tf_: x = %f, y = %f", laser_to_base_tf_.getOrigin().getX(), laser_to_base_tf_.getOrigin().getY());
}

/*
  convert data from laser coordinate to base coordinate
*/
void ScanManager::voxelFilterCloudToBase(std::vector<Eigen::Vector3f>& filtered_point_cloud)
{
//   dynamic_map_ptr.setLaserToBaseOffset(x, y);

  int num = filtered_point_cloud.size();
  for (int i = 0; i < num; ++i) {
   filtered_point_cloud[i].x() += laser_to_base_tf_.getOrigin().getX();
   filtered_point_cloud[i].y() += laser_to_base_tf_.getOrigin().getY();

   if(i % 10 == 0 || i == num - 1)
   {
     ROS_INFO("After undistortion: index = %d, x = %f, y = %f", i, filtered_point_cloud[i].x(), filtered_point_cloud[i].y());
   }

  }
  
  return;
}

void ScanManager::laserAngleToBase(const sensor_msgs::LaserScan::ConstPtr &laser_scan, double &angle_min, double &angle_increment, const std::shared_ptr<tf2_ros::Buffer>& tf)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, laser_scan->angle_min);
  geometry_msgs::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = laser_scan->header.stamp;
  min_q.header.frame_id = laser_scan->header.frame_id;
  tf2::convert(q, min_q.quaternion);

  q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
  inc_q.header = min_q.header;
  tf2::convert(q, inc_q.quaternion);
  try {
    tf->transform(min_q, min_q, base_frame_id_);
    tf->transform(inc_q, inc_q, base_frame_id_);
  }
  catch (tf2::TransformException &e) {
    ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
             e.what());
    return;
  }

  angle_min = tf2::getYaw(min_q.quaternion);
  angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // ROS_INFO("laser angle_min = %f", angle_min);
  // ROS_INFO("laser angle_increment = %f", angle_increment);

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;
} 


void ScanManager::scanMsgToMapPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud, pf_vector_t base_pose)
{
    int length = scan_cloud->points.size();

    for(int i = 0; i < length; i++)
    {
        float x = scan_cloud->points[i].x;
        float y = scan_cloud->points[i].y;
        scan_cloud->points[i].x = cos(base_pose.v[2]) * x - sin(base_pose.v[2]) * y + base_pose.v[0];
        scan_cloud->points[i].y = sin(base_pose.v[2]) * x + cos(base_pose.v[2]) * y + base_pose.v[1];
        scan_cloud->points[i].z = 0;
    }  
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ScanManager::eigenCloudToPCL(const std::vector<Eigen::Vector3f>& point_cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl_cloud->width = point_cloud.size();
  pcl_cloud->height = 1;
  pcl_cloud->resize(point_cloud.size());
  sensor_msgs::PointCloud2 filter_cloud;

  for (int i = 0; i < point_cloud.size(); ++i) {
    pcl_cloud->points[i].x = point_cloud[i].x();
    pcl_cloud->points[i].y = point_cloud[i].y();
    pcl_cloud->points[i].z = 0;
  }

  return pcl_cloud;
}

std::vector<Eigen::Vector3f>  ScanManager::scanDataGravityAlign(const std::vector<Eigen::Vector3f>& laser_points, Eigen::Quaternionf rot)
{
  std::vector<Eigen::Vector3f> gravity_align_points;

  Eigen::Vector3f laser_base_offset(laser_to_base_tf_.getOrigin().getX(), laser_to_base_tf_.getOrigin().getY(), 0);

  int length = laser_points.size();

  for(int i = 0; i < length; i++)
  {
    Eigen::Vector3f tmp_point;

    tmp_point = rot * (laser_points[i] + laser_base_offset);

    gravity_align_points.push_back(tmp_point);
  }

  return gravity_align_points;
}


std::vector<double> ScanManager::getRanges()
{
  return ranges_;
}

std::vector<double> ScanManager::getAngles()
{
  return angles_;
}


tf2::Transform ScanManager::getLaserLaserToBaseOffset()
{
  return laser_to_base_tf_;
}

void ScanManager::setLaserToBaseOffset(const tf2::Transform& laser_to_base)
{
  laser_to_base_tf_ = laser_to_base;
}

