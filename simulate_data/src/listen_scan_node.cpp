#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <string>

class ListenScan
{
public:
    ListenScan();
    ~ListenScan();

    void scan_cb(sensor_msgs::LaserScan::ConstPtr msg); 
    void getTransformFromLaserToBase(const sensor_msgs::LaserScan::ConstPtr msg);

    void laserAngleToBase(const sensor_msgs::LaserScan::ConstPtr &laser_scan,
                          double &angle_min,
                          double &angle_increment);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber scan_sub_;  


    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf_;

    std::string base_frame_id_;

    bool first_rec_flag_;

};

ListenScan::ListenScan(): private_nh_("~"), first_rec_flag_(false)
{
    tf_.reset(new tf2_ros::Buffer());
    tfl_.reset(new tf2_ros::TransformListener(*tf_));

    private_nh_.param<std::string>("base_frame_id",base_frame_id_, "base_link");

    scan_sub_ = nh_.subscribe("scan", 50, &ListenScan::scan_cb, this);
}

ListenScan::~ListenScan()
{

}

void ListenScan::scan_cb(sensor_msgs::LaserScan::ConstPtr msg)
{
  if(first_rec_flag_)
  {
    return;
  }
  double angle_min = 0, angle_increment = 0;
  laserAngleToBase(msg, angle_min, angle_increment);

  ROS_INFO_ONCE("laser angle_min = %f, laser angle_increment = %f", msg->angle_min, msg->angle_increment);
  ROS_INFO_ONCE("angle_min = %f, angle_increment = %f", angle_min, angle_increment);

  getTransformFromLaserToBase(msg);

  first_rec_flag_ = true;
}

void ListenScan::laserAngleToBase(const sensor_msgs::LaserScan::ConstPtr &laser_scan,
                                double &angle_min,
                                double &angle_increment) {
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
    tf_->transform(min_q, min_q, base_frame_id_);
    tf_->transform(inc_q, inc_q, base_frame_id_);
  }
  catch (tf2::TransformException &e) {
    ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
             e.what());
    return;
  }

  angle_min = tf2::getYaw(min_q.quaternion);
  angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;
}

void ListenScan::getTransformFromLaserToBase(const sensor_msgs::LaserScan::ConstPtr msg)
{
  geometry_msgs::TransformStamped tx_odom;
  try {
    ros::Time now = ros::Time::now();
    // wait a little for the latest tf to become available
    tx_odom = tf_->lookupTransform(base_frame_id_, msg->header.frame_id, ros::Time::now(),
                                   ros::Duration(0.5));
  }
  catch (tf2::TransformException e) {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    ROS_ERROR("Cannot get transform!!!"); 
    tf2::convert(tf2::Transform::getIdentity(), tx_odom.transform);
  }

  tf2::Quaternion quat_tf;
  tf2::convert(tx_odom.transform.rotation, quat_tf);

  tf2::Vector3 col_0 =  tf2::Matrix3x3(quat_tf).getColumn(0);
  tf2::Vector3 col_1 =  tf2::Matrix3x3(quat_tf).getColumn(1);
  tf2::Vector3 col_2 =  tf2::Matrix3x3(quat_tf).getColumn(2);

  std::cout << col_0.getX() << ' ' << col_1.getX() << ' ' << col_2.getX() << std::endl;
  std::cout << col_0.getY() << ' ' << col_1.getY() << ' ' << col_2.getY() << std::endl;
  std::cout << col_0.getZ() << ' ' << col_1.getZ() << ' ' << col_2.getZ() << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_publisher");

    ListenScan listen_scan;

    ros::spin();
    return 0;
}
