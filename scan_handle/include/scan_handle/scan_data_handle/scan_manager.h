#ifndef SCAN_MANAGER_H
#define SCAN_MANAGER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "scan_handle/lidar_undistortion/lidar_undistortion.h"
#include "scan_handle/config.h"
#include "scan_handle/utils.h"
#include "tf2_ros/transform_listener.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>


class ScanManager{
  public:
    ScanManager();
    ~ScanManager();

    ScanManager(const ScanManager&) = delete;
    ScanManager& operator=(const ScanManager&) = delete;

    void setOptions(Config& config);

    void scanDataCollate(const sensor_msgs::LaserScan::ConstPtr&laser_scan, const double& angle_min, const double& angle_increment);
    void scanDataTimeHandle(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
    void scanDataUndistortion();
    std::vector<Eigen::Vector3f> scanDataToCoordinate();
    void calculateLaserToBaseOffset(const std::shared_ptr<tf2_ros::Buffer>& tf);
    void laserAngleToBase(const sensor_msgs::LaserScan::ConstPtr &laser_scan, double &angle_min, double &angle_increment, const std::shared_ptr<tf2_ros::Buffer>& tf);
    void voxelFilterCloudToBase(std::vector<Eigen::Vector3f>& filtered_point_cloud);
    
    std::vector<Eigen::Vector3f> scanDataGravityAlign(const std::vector<Eigen::Vector3f>& laser_points, Eigen::Quaternionf rot);

    pcl::PointCloud<pcl::PointXYZ>::Ptr eigenCloudToPCL(const std::vector<Eigen::Vector3f>& point_cloud);

    void scanMsgToMapPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud, pf_vector_t base_pose);

    std::vector<double> getRanges();
    std::vector<double> getAngles();
    tf2::Transform getLaserLaserToBaseOffset();
    void setLaserToBaseOffset(const tf2::Transform& laser_to_base);

  private:
    LidarMotionCalibrator lidar_motion_calibrator_;
    std::string scan_frame_id_;
    ros::Time start_time_;
    ros::Time end_time_;
    double increment_time_;
    std::vector<double> ranges_;
    std::vector<double> angles_;
    tf2::Transform laser_to_base_tf_;
    float laser_min_range_; 
    float laser_max_range_;
    bool scan_time_from_start_;
    std::string base_frame_id_;

};


#endif
