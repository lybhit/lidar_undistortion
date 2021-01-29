#include <string>
#include <map>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <tf/transform_broadcaster.h>

#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "sensor_msgs/MagneticField.h"


class ImuNode
{
 public:
    ImuNode() : private_nh_("~")
    {
        // Get node parameters

        private_nh_.param("rate", rate_, 200);
        private_nh_.param<std::string>("frame_id", frame_id_, "imu_link");

        // Timestamp synchronization
        private_nh_.param("angular_z", angular_z_, 0.1);

        // Connect to the LP IMU device

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data",1);

    }

    ~ImuNode(void)
    {
    }

    void update(const ros::TimerEvent& te)
    {

            /* Fill the IMU message */

            // Fill the header
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = frame_id_;

            // Fill orientation quaternion
            double delta_time = (ros::Time::now() - last_pub_time_).toSec();
            last_pub_time_ = ros::Time::now();

            angle_val_ += angular_z_ * delta_time;

            if(angle_val_ > M_1_PI)
            {
                angle_val_ -= 2 * M_1_PI;
            }else if(angle_val_ < -M_1_PI)
            {
                angle_val_ += 2 * M_1_PI;
            }

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle_val_);
            imu_msg.orientation.w = odom_quat.w;
            imu_msg.orientation.x = odom_quat.x;
            imu_msg.orientation.y = odom_quat.y;
            imu_msg.orientation.z = odom_quat.z;

            // Fill angular velocity data
            // - scale from deg/s to rad/s
            imu_msg.angular_velocity.x = 0.0;
            imu_msg.angular_velocity.y = 0.0;
            imu_msg.angular_velocity.z = angular_z_;

            // Fill linear acceleration data
            imu_msg.linear_acceleration.x = 0;
            imu_msg.linear_acceleration.y = 0;
            imu_msg.linear_acceleration.z = 0;

            // \TODO: Fill covariance matrices
            // msg.orientation_covariance = ...
            // msg.angular_velocity_covariance = ...
            // msg linear_acceleration_covariance = ...

            // Publish the messages
            imu_pub_.publish(imu_msg);
    }

    void run(void)
    {
        // The timer ensures periodic data publishing
        updateTimer_ = ros::Timer(nh_.createTimer(ros::Duration(0.1/rate_),
                                                &ImuNode::update,
                                                this));
    }

 private:

    // Access to ROS node
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer updateTimer_;
    ros::Publisher imu_pub_;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;

    double angular_z_;
    double angle_val_;
    ros::Time current_time_, last_pub_time_;

    // Parameters
    std::string frame_id_;
    int rate_;

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simulate_imu");

    ImuNode imu_node;
    imu_node.run();

    ros::spin();

    return 0;
}