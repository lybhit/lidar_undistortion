#include "scan_handle/scan_test.h"

inline
std::string stripSlash(const std::string& in)
{
  std::string out = in;
  if ( ( !in.empty() ) && (in[0] == '/') )
    out.erase(0,1);
  return out;
}


ScanTest::ScanTest(){

  tfb_.reset(new tf2_ros::TransformBroadcaster());
  tf_.reset(new tf2_ros::Buffer(ros::Duration(10)));
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud", 100, true);

  laser_scan_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 100));
  laser_scan_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                         *tf_,
                                                         config_.odom_frame_id,
                                                         100,
                                                         nh_));
  laser_scan_filter_->registerCallback(boost::bind(&ScanTest::laserReceived, this, _1));

  odom_sub_ = nh_.subscribe("odom", 10, &ScanTest::odomReceived, this);

  scan_manager_.setOptions(config_);
  frame_to_laser_.clear();
}

ScanTest::~ScanTest()
{

}


void
ScanTest::odomReceived(const nav_msgs::Odometry &raw_msg) {
  geometry_msgs::TransformStamped odom_trans_;
  tf2::Quaternion q_orig;
  tf2::convert(raw_msg.pose.pose.orientation, q_orig);
  odom_trans_.header.stamp = raw_msg.header.stamp;
  odom_trans_.header.frame_id = "odom";
  odom_trans_.child_frame_id = config_.base_frame_id;
  odom_trans_.transform.translation.x = raw_msg.pose.pose.position.x;
  odom_trans_.transform.translation.y = raw_msg.pose.pose.position.y;
  odom_trans_.transform.translation.z = raw_msg.pose.pose.position.z;

  odom_trans_.transform.rotation.x = q_orig.x();
  odom_trans_.transform.rotation.y = q_orig.y();
  odom_trans_.transform.rotation.z = q_orig.z();
  odom_trans_.transform.rotation.w = q_orig.w();

  // ROS_INFO("latest odom stamp = %f", raw_msg.header.stamp.toSec());
  // odom_time_ = raw_msg.header.stamp.toSec();

  tfb_->sendTransform(odom_trans_);
}

void ScanTest::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  static int counter = 0;
  counter++;
  if(counter < 10)
  {
    return;
  }

  std::string laser_scan_frame_id = stripSlash(laser_scan->header.frame_id);

  int laser_index = -1;

  if(frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan_frame_id.c_str());
    laser_index = frame_to_laser_.size();

    geometry_msgs::PoseStamped ident;
    ident.header.frame_id = laser_scan_frame_id;
    ident.header.stamp = ros::Time();
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

    tf2::Transform laser_2_base;

    geometry_msgs::PoseStamped laser_pose;
    try
    {
      this->tf_->transform(ident, laser_pose, config_.base_frame_id);
    }
    catch(tf2::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan_frame_id.c_str(),
                config_.base_frame_id.c_str());
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.pose.position.x;
    laser_pose_v.v[1] = laser_pose.pose.position.y;
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan_frame_id] = laser_index;

    laser_2_base.setOrigin(tf2::Vector3(laser_pose_v.v[0], laser_pose_v.v[1],0));
    laser_2_base.setBasis(tf2::Matrix3x3::getIdentity());
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan_frame_id];
  }

  double angle_min, angle_increment;

  scan_manager_.laserAngleToBase(laser_scan, angle_min, angle_increment, tf_);
  scan_manager_.scanDataCollate(laser_scan, angle_min, angle_increment);
  scan_manager_.scanDataTimeHandle(laser_scan);
  std::vector<Eigen::Vector3f> coordinate_points_before = scan_manager_.scanDataToCoordinate();
  ROS_INFO("*******");
  scan_manager_.scanDataUndistortion();
  std::vector<Eigen::Vector3f> coordinate_points = scan_manager_.scanDataToCoordinate();
  scan_manager_.voxelFilterCloudToBase(coordinate_points);
  ROS_INFO("*******");

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud = scan_manager_.eigenCloudToPCL(coordinate_points);
  scan_cloud->header.frame_id = config_.base_frame_id;
  scan_cloud_pub_.publish(scan_cloud);

}