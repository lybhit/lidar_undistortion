#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

class OdomNode
{
public:
  OdomNode();
  ~OdomNode();

  void publishOdomMsg(const ros::TimerEvent& te);
  void run(void);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;

  double vx_;
  double vy_;
  double vth_;

  double x_;
  double y_;
  double theta_;

  ros::Time current_time_, last_time_;
  ros::Timer updateTimer_;
  int rate_;
};

OdomNode::OdomNode():private_nh_("~"), x_(0), y_(0), theta_(0)
{
    private_nh_.param("rate", rate_, 50);
    private_nh_.param("vx", vx_, 1.0);
    private_nh_.param("vy", vy_, 0.0);
    private_nh_.param("vth", vth_, 0.0);
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
}

OdomNode::~OdomNode()
{

}

void OdomNode::publishOdomMsg(const ros::TimerEvent& te)
{    
    current_time_ = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time_ - last_time_).toSec();
    double delta_x = (vx_ * cos(theta_) - vy_ * sin(theta_)) * dt;
    double delta_y = (vx_ * sin(theta_) + vy_ * cos(theta_)) * dt;
    double delta_th = vth_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster_.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vth_;

    //publish the message
    odom_pub_.publish(odom);

    last_time_ = current_time_;
}

void OdomNode::run(void)
{
    // The timer ensures periodic data publishing
    updateTimer_ = ros::Timer(nh_.createTimer(ros::Duration(1/rate_),
                                            &OdomNode::publishOdomMsg,
                                            this));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry");

  OdomNode odom_node;
  odom_node.run();

  ros::spin();

  return 0;
}