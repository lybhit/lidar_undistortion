#ifndef __LIDAR_UNDISTORTION_H__
#define __LIDAR_UNDISTORTION_H__

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <string>
#include <memory>


class LidarMotionCalibrator
{
public:
    //构造函数
    LidarMotionCalibrator(std::string scan_frame_name,std::string base_frame_name, std::string odom_frame_name);
    //析构函数，释放tf_
    ~LidarMotionCalibrator();
    //激光雷达运动畸变去除函数
    void lidarCalibration(std::vector<double>& ranges, std::vector<double>& angles, ros::Time startTime, const ros::Time endTime, const double time_inc);

    //从tf缓存数据中，寻找对应时间戳的里程计位姿
    bool getLaserPose(geometry_msgs::PoseStamped & odom_pose, ros::Time dt);
    //根据传入参数，对任意一个分段进行插值
    void lidarMotionCalibration(geometry_msgs::PoseStamped frame_target_pose, geometry_msgs::PoseStamped frame_source_pose,
                                 std::vector<double>& ranges, std::vector<double>& angles, int startIndex);
public:
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string scan_frame_name_;
    std::string odom_frame_name_;
    std::string base_frame_name_;
};

#endif