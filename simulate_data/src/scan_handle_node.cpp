#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

#include <math.h>
#include <string>

class ScanHandle
{
public:
    ScanHandle();
    ~ScanHandle();
    
    void publishScanMsg();
    int getPubFrequency();

    void scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher scan_pub_;  
    ros::Subscriber scan_sub_;

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf_;

    int laser_frequency_;
    int num_readings_;
    double range_;

    bool first_receive_flag_;
    std::string base_frame_id_;
};

ScanHandle::ScanHandle(): private_nh_("~"), first_receive_flag_(false)
{
    private_nh_.param("laser_frequency", laser_frequency_, 20);
    private_nh_.param("num_readings", num_readings_, 100);
    private_nh_.param("range", range_, 50.0);
    private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");

    tf_.reset(new tf2_ros::Buffer());
    tfl_.reset(new tf2_ros::TransformListener(*tf_));
    
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_", 50);
    scan_sub_ = nh_.subscribe("scan", 10, &ScanHandle::scan_cb, this);

}

ScanHandle::~ScanHandle()
{

}

int ScanHandle::getPubFrequency()
{
    return laser_frequency_;
}

void ScanHandle::publishScanMsg()
{
    double ranges[num_readings_];
    double intensities[num_readings_];

    for(unsigned int i = 0; i < num_readings_; ++i)
    {
        ranges[i] = range_;
        intensities[i] = 100 + range_;
    }
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_link";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings_;
    scan.time_increment = (1. / laser_frequency_) / (num_readings_);
    scan.range_min = 0.0;
    scan.range_max = 100.0;
    scan.ranges.resize(num_readings_);
    scan.intensities.resize(num_readings_);
    for(unsigned int i = 0; i < num_readings_; ++i)
    {
        scan.ranges[i] = ranges[i];
        scan.intensities[i] = intensities[i];
    }

    scan_pub_.publish(scan);
    
}

void ScanHandle::scan_cb(const sensor_msgs::LaserScanConstPtr& msg)
{
  if(first_receive_flag_)
  {
    return;
  }

  geometry_msgs::TransformStamped laser_to_base;

  laser_to_base = tf_->lookupTransform(base_frame_id_, msg->header.frame_id, ros::Time(0));
  tf2::Stamped<tf2::Transform> stamped_transform;
  tf2::fromMsg(laser_to_base, stamped_transform);

  tf2::Vector3 a = stamped_transform.getBasis().getColumn(0);
  tf2::Vector3 b = stamped_transform.getBasis().getColumn(1);
  tf2::Vector3 c = stamped_transform.getBasis().getColumn(2);

  std::cout << a[0] << ' ' << b[0] << ' ' << c[0] << std::endl; 
  std::cout << a[1] << ' ' << b[1] << ' ' << c[1] << std::endl; 
  std::cout << a[2] << ' ' << b[2] << ' ' << c[2] << std::endl; 

  double angle_min, angle_increment;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, msg->angle_min);
  geometry_msgs::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = msg->header.stamp;
  min_q.header.frame_id = msg->header.frame_id;
  tf2::convert(q, min_q.quaternion);

  q.setRPY(0.0, 0.0, msg->angle_min + msg->angle_increment);
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
  ROS_INFO("Laser's min_angle and angle_increment in base_link frame = %f, %f", angle_min, angle_increment);

  first_receive_flag_ = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_handle_node");

    ScanHandle scan_handle;

    ros::spin();
    return 0;
}
