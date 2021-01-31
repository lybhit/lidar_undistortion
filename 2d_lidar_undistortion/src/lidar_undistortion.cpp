#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
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

using namespace std;
using namespace chrono;

typedef boost::circular_buffer<sensor_msgs::Imu> ImuCircularBuffer;

class LidarMotionCalibrator
{
public:

    LidarMotionCalibrator()
    {
      ros::NodeHandle nh_param("~");

      nh_param.param<string>("lidar_topic", lidar_topic_, "/scan");
      nh_param.param<string>("imu_topic", imu_topic_, "/mti/sensor/imu");
      nh_param.param<double>("imu_frequency", imu_frequency_, 100.0);
      nh_param.param<double>("lidar_msg_delay_time", lidar_msg_delay_time_, 10.0);

      nh_param.param<string>("output_frame_id", output_frame_id_, "laser");
      nh_param.param<bool>("pub_raw_scan_pointcloud", pub_raw_scan_pointcloud_, false);
      nh_param.param<bool>("pub_laserscan", pub_laserscan_, false);
      nh_param.param<double>("laserscan_angle_increment", laserscan_angle_increment_, 0.01);

      nh_param.param<bool>("scan_direction_clockwise", scan_direction_clockwise_, false);

      nh_param.param<bool>("use_range_filter", use_range_filter_, false);
      nh_param.param<double>("range_filter_min", range_filter_min_, 0.3);
      nh_param.param<double>("range_filter_max", range_filter_max_, 12.0);

      nh_param.param<bool>("use_angle_filter", use_angle_filter_, false);
      nh_param.param<double>("angle_filter_min", angle_filter_min_, -2.5);
      nh_param.param<double>("angle_filter_max", angle_filter_max_,  2.5);

      nh_param.param<bool>("use_radius_outlier_filter", use_radius_outlier_filter_, false);
      nh_param.param<double>("radius_outlier_filter_search_radius", radius_outlier_filter_search_radius_, 0.1);
      nh_param.param<int>("radius_outlier_filter_min_neighbors", radius_outlier_filter_min_neighbors_, 1);


      lidar_sub_ = nh_.subscribe(lidar_topic_, 10, &LidarMotionCalibrator::LidarCallback, this);
      imu_sub_ = nh_.subscribe(imu_topic_, 100, &LidarMotionCalibrator::ImuCallback, this);

      pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/lidar_undistortion/after", 10);
      pcl_pub_origin_ = nh_.advertise<sensor_msgs::PointCloud2> ("/lidar_undistortion/origin", 10);
      laserscan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/lidar_undistortion/scan", 10);

      delay_duration = ros::Duration(lidar_msg_delay_time_ / 1000.0);

      imuCircularBuffer_ = ImuCircularBuffer((int)(imu_frequency_ * 4));
    }


    ~LidarMotionCalibrator()
    {

    }

    void ImuCallback(const sensor_msgs::ImuConstPtr& _imu_msg)
    {
      imuCircularBuffer_.push_front(*_imu_msg);
    }

    void LidarCallback(const sensor_msgs::LaserScanConstPtr& _lidar_msg)
    {
      // 激光点的个数
      int length = _lidar_msg->ranges.size();
      ROS_INFO("scan range size = %d", length);

      // 当前激光帧从头到尾的时间
      // The overall scan time of current laserscan message(duration from the first scan point to the last).
      if(_lidar_msg->scan_time != 0)
      {
        frame_duration = ros::Duration(_lidar_msg->scan_time);
      }else{
        ROS_INFO("scan time = %f", _lidar_msg->scan_time);
        frame_duration = ros::Duration(_lidar_msg->time_increment * (length - 1));
      }

      // 估计第一个激光点的时刻
      // Estimate the timestamp of the first scan point.
      // ros::Time lidar_timebase = ros::Time::now() - frame_duration - delay_duration;
      // ros::Time lidar_timebase =  _lidar_msg->header.stamp;
      ros::Time lidar_timebase =  _lidar_msg->header.stamp - frame_duration;
      pcl::PointCloud<pcl::PointXYZI> pointcloud_raw_pcl;

      // 将LaserScan类型的数据转化为PCL点云格式
      // Change sensor_msgs::LaserScan format to PCL pointcloud format.
      LaserScanToPointCloud(_lidar_msg, pointcloud_raw_pcl);

      current_laserscan_msg_ = _lidar_msg;

      // 保证imu buffer中有足够多的数据
      // Ensure that there is sufficient Imu data stored in imuCircularBuffer_.
      if (imuCircularBuffer_.size() > frame_duration.toSec() * imu_frequency_ * 1.5)
      {
        auto start = system_clock::now();

        sensor_msgs::PointCloud2 pointcloud_msg;
        sensor_msgs::PointCloud2 pointcloud_raw_msg;
        pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
        pcl::PointXYZI point_xyzi;

        current_output_timestamp = imuCircularBuffer_[0].header.stamp;
        Eigen::Quaternionf current_quat = ImuToCurrentQuaternion(imuCircularBuffer_[0].orientation);
        Eigen::Vector3f eulerAngle= current_quat.matrix().eulerAngles(2,1,0);
        std::cout << "start pose angle = " << eulerAngle << std::endl;

        for (int i = 0; i < length; i++)
        {
          pcl::PointXYZI current_point = pointcloud_raw_pcl[i];

          Eigen::Vector3f point_in(current_point.x, current_point.y, 0.0);

          ros::Time point_timestamp = lidar_timebase + ros::Duration(_lidar_msg->time_increment * i);

          Eigen::Quaternionf point_quat;

          // 如果成功获取当前扫描点的姿态
          if (getLaserPose(i, point_timestamp, point_quat) == true)
          {

            // Eigen::Quaternionf delta_quat = current_quat.inverse() * point_quat;
            Eigen::Quaternionf delta_quat = current_quat.conjugate() * point_quat;
            Eigen::Vector3f deltaAngle= delta_quat.matrix().eulerAngles(2,1,0);
            Eigen::Vector3f current_inv_angle= current_quat.conjugate().matrix().eulerAngles(2,1,0);

            
            if(i % 10 == 0)
            {
              Eigen::Vector3f eulerAngle_1 = point_quat.matrix().eulerAngles(2,1,0);
              std::cout << "current inverse angle = " << current_inv_angle << std::endl;

              std::cout << "index " << i << " angle = " << eulerAngle_1 << std::endl;
              std::cout << "delta angle = " << deltaAngle << std::endl;
            }

            Eigen::Vector3f point_out = delta_quat * point_in;

            point_xyzi.x = point_out(0);
            point_xyzi.y = point_out(1);
            point_xyzi.z = 0.0;
            point_xyzi.intensity = current_point.intensity;
            pointcloud_pcl.push_back(point_xyzi);
          }
          else
          {
            break;
          }
        }

        ROS_INFO("point cloud size = %d", pointcloud_pcl.size());

        // ApplyRangeFilter(pointcloud_pcl);
        // ApplyAngleFilter(pointcloud_pcl);
        // ApplyRadiusOutlierFilter(pointcloud_pcl);

        pcl::toROSMsg(pointcloud_pcl, pointcloud_msg);
        pointcloud_msg.header.frame_id = output_frame_id_;
        pcl_pub_.publish(pointcloud_msg);

        if (pub_raw_scan_pointcloud_ == true)
        {
          pcl::toROSMsg(pointcloud_raw_pcl, pointcloud_raw_msg);
          pointcloud_raw_msg.header.frame_id = output_frame_id_;
          pcl_pub_origin_.publish(pointcloud_raw_msg);
        }

        if (pub_laserscan_ == true)
        {
          PublishLaserscan(pointcloud_pcl);
        }

        auto end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        cout <<  "Spend "  << double(duration.count()) / 1000.0 << " miliseconds" << endl;
      }
      else
      {
        cout << "imu data less than 0.2s, waiting for next packet" << endl;
      }
    }

    /**
     * @name getLaserPose()
     * @brief 得到机器人在当前激光点的姿态，返回是否成功
     * @param point_index  激光点的序号
     * @param _timestamp   当前激光点的时间戳
     * @param quat_out  输出四元数
    */
    bool getLaserPose(int point_index, ros::Time &_timestamp, Eigen::Quaternionf &quat_out)
    {
      // cout << "point_index = " << point_index << endl;
      static int index_front = 0;
      static int index_back = 0;
      static ros::Time timestamp_front;
      static ros::Time timestamp_back;
      static Eigen::Quaternionf quat_front, quat_back;

      static bool index_updated_flag = false;
      static bool predict_orientation_flag = false;

      if (point_index == 0)
      {
        predict_orientation_flag = false;
        int i = 0;
        while (_timestamp < imuCircularBuffer_[i].header.stamp)
        {
          i++;
        }
        index_front = i - 1;
        index_back = i;
        index_updated_flag = true;
      }
      else
      {
        while (predict_orientation_flag == false
               && _timestamp > imuCircularBuffer_[index_front].header.stamp
               && _timestamp > imuCircularBuffer_[index_back].header.stamp)
        {
          index_front--;
          index_back--;

          if (index_front < 0)
          {
            //use prediction
            predict_orientation_flag = true;
            index_front++;
            index_back++;
          }

          index_updated_flag = true;
        }
      }

      if (index_updated_flag == true)
      {
        cout << "index updated: " << index_front << " " << index_back << endl;
        timestamp_front = imuCircularBuffer_[index_front].header.stamp;
        timestamp_back = imuCircularBuffer_[index_back].header.stamp;
        quat_front = Eigen::Quaternionf(imuCircularBuffer_[index_front].orientation.w, imuCircularBuffer_[index_front].orientation.x, imuCircularBuffer_[index_front].orientation.y, imuCircularBuffer_[index_front].orientation.z);
        quat_back = Eigen::Quaternionf(imuCircularBuffer_[index_back].orientation.w, imuCircularBuffer_[index_back].orientation.x, imuCircularBuffer_[index_back].orientation.y, imuCircularBuffer_[index_back].orientation.z);

        index_updated_flag = false;
      }

      float alpha = (float)(_timestamp.toNSec() - timestamp_back.toNSec()) / (timestamp_front.toNSec() - timestamp_back.toNSec());
      if(point_index % 10 == 0)
      {
        std::cout << "slerp alpha = " << alpha << std::endl;
      }

      if (alpha < 0)
      {
        return false;
      }

      // 球面线性插值
      // Slerp.
      quat_out = quat_back.slerp(alpha, quat_front);

      return true;
    }

    void LaserScanToPointCloud(sensor_msgs::LaserScan::ConstPtr _laser_scan, pcl::PointCloud<pcl::PointXYZI>& _pointcloud)
    {
      ROS_INFO("laser scan to pointcloud.");
      _pointcloud.clear();
      pcl::PointXYZI newPoint;
      newPoint.z = 0.0;
      double newPointAngle;

      int beamNum = _laser_scan->ranges.size();
      
      //如果设置顺时针，则读取数据顺序为从scan数据中从n -> 0
      if (scan_direction_clockwise_ == true)
      {
        for (int i = beamNum - 1; i >= 0; i--)
        {
          newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;
          newPoint.x = _laser_scan->ranges[i] * cos(newPointAngle);
          newPoint.y = _laser_scan->ranges[i] * sin(newPointAngle);
          newPoint.intensity = _laser_scan->intensities[i];
          _pointcloud.push_back(newPoint);
        }
      }
      //如果设置逆时针，则读取数据顺序为从scan数据中从0 -> n
      else
      {
        for (int i = 0; i < beamNum; i++)
        {
          newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;
          newPoint.x = _laser_scan->ranges[i] * cos(newPointAngle);
          newPoint.y = _laser_scan->ranges[i] * sin(newPointAngle);
          newPoint.intensity = _laser_scan->intensities[i];
          _pointcloud.push_back(newPoint);
        }
      }
    }

    Eigen::Quaternionf ImuToCurrentQuaternion(geometry_msgs::Quaternion _quat)
    {
      tf::Quaternion current_tf_quat(_quat.x, _quat.y, _quat.z, _quat.w);

      //提取航向角
      double roll, pitch, yaw;
      tf::Matrix3x3(current_tf_quat).getRPY(roll, pitch, yaw);

      //建立当前航向的水平坐标系四元数
      geometry_msgs::Quaternion horizontal_quat = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);

      return Eigen::Quaternionf(horizontal_quat.w, horizontal_quat.x, horizontal_quat.y, horizontal_quat.z);
    }

    void PublishLaserscan(pcl::PointCloud<pcl::PointXYZI>& _pointcloud)
    {
      float angle_min, angle_max;
      if (use_angle_filter_ == true)
      {
        angle_min = angle_filter_min_;
        angle_max = angle_filter_max_;
      }
      else
      {
        angle_min = current_laserscan_msg_->angle_min;
        angle_max = current_laserscan_msg_->angle_max;
      }

      float range_min, range_max;
      if (use_range_filter_ == true)
      {
        range_min = range_filter_min_;
        range_max = range_filter_max_;
      }
      else
      {
        range_min = current_laserscan_msg_->range_min;
        range_max = current_laserscan_msg_->range_max;
      }

      unsigned int beam_size = ceil((angle_max - angle_min) / laserscan_angle_increment_);
      ROS_INFO("if publish scan, beam size = %d", beam_size);

      sensor_msgs::LaserScan output;
      output.header.stamp = current_output_timestamp;
      output.header.frame_id = output_frame_id_;
      output.angle_min = angle_min;
      output.angle_max = angle_max;
      output.range_min = range_min;
      output.range_max = range_max;
      output.angle_increment = laserscan_angle_increment_;
      output.time_increment = 0.0;
      output.scan_time = 0.0;
      output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
      output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

      for (auto point : _pointcloud.points)
      {
        float range = hypot(point.x, point.y);
        float angle = atan2(point.y, point.x);
        int index = (int)((angle - output.angle_min) / output.angle_increment);
        if (index >= 0 && index < beam_size)
        {
          if (isnan(output.ranges[index]))
          {
            output.ranges[index] = range;
          }
          else
          {
            if (range < output.ranges[index])
            {
              output.ranges[index] = range;
            }
          }
          output.intensities[index] = point.intensity;
        }
      }
      laserscan_pub_.publish(output);
    }

    void ApplyRangeFilter(pcl::PointCloud<pcl::PointXYZI>& _input)
    {
      if (use_range_filter_ == true)
      {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        for (int i = 0; i < _input.size(); i++)
        {
          float range = hypot(_input[i].x, _input[i].y);
          if (range > range_filter_min_ && range < range_filter_max_)
          {
            cloud.push_back(_input[i]);
          }
        }
        _input.swap(cloud);
      }
    }

    void ApplyAngleFilter(pcl::PointCloud<pcl::PointXYZI>& _input)
    {
      if (use_angle_filter_ == true)
      {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        for (int i = 0; i < _input.size(); i++)
        {
          float angle = atan2(_input[i].y, _input[i].x);
          if (angle > angle_filter_min_ && angle < angle_filter_max_)
          {
            cloud.push_back(_input[i]);
          }
        }
        _input.swap(cloud);
      }
    }

    void ApplyRadiusOutlierFilter(pcl::PointCloud<pcl::PointXYZI>& _input)
    {
      if (use_radius_outlier_filter_ == true)
      {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(_input.makeShared());
        sor.setRadiusSearch(radius_outlier_filter_search_radius_);
        sor.setMinNeighborsInRadius(radius_outlier_filter_min_neighbors_);
        sor.setNegative(false);
        sor.filter(cloud);
        _input.swap(cloud);
      }
    }

public:

    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher pcl_pub_;
    ros::Publisher pcl_pub_origin_;
    ros::Publisher laserscan_pub_;

    ros::Duration frame_duration, delay_duration;
    ros::Time current_output_timestamp;

    ImuCircularBuffer imuCircularBuffer_;

    string lidar_topic_, imu_topic_, output_frame_id_;
    double imu_frequency_, lidar_msg_delay_time_;
    bool pub_raw_scan_pointcloud_;

    bool pub_laserscan_;
    double laserscan_angle_increment_;

    bool use_range_filter_;
    double range_filter_min_;
    double range_filter_max_;
    bool scan_direction_clockwise_;

    bool use_angle_filter_;
    double angle_filter_min_;
    double angle_filter_max_;

    bool use_radius_outlier_filter_;
    double radius_outlier_filter_search_radius_;
    int radius_outlier_filter_min_neighbors_;

    sensor_msgs::LaserScan::ConstPtr current_laserscan_msg_;

};


int main(int argc,char ** argv)
{
    ros::init(argc, argv, "lidar_undistortion");

    LidarMotionCalibrator myLidarMotionCalibrator;

    ros::spin();

    return 0;
}
