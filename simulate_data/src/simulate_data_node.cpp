#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <Eigen/Dense>
#include <iostream>
#include <dirent.h>
#include <fstream>
#include <cmath>
#include <chrono>

#include <boost/circular_buffer.hpp>

class SimulateData
{
public:
  SimulateData();
  ~SimulateData();

  void publishScanMsg();
  void publishImuMsg();
  




private:
  ros::Publisher scan_pub_;
  ros::Publisher imu_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;



};

