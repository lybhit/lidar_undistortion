#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include "message_filters/subscriber.h"
#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <string>
#include <memory>
//如果使用调试模式，可视化点云，需要安装PCL
#define debug_ 1

#if debug_
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");//初始化一个pcl窗口
#endif

//雷达运动畸变去除类
class LidarMotionCalibrator
{
public:
    //构造函数，初始化tf_、订阅者、回调函数ScanCallBack
    LidarMotionCalibrator();
    //析构函数，释放tf_
    ~LidarMotionCalibrator();
    //拿到原始的激光数据来进行处理
    void ScanCallBack(const sensor_msgs::LaserScanConstPtr& scan_msg);
    //激光雷达运动畸变去除函数

    void lidarCalibration(std::vector<double>& ranges,std::vector<double>& angles,ros::Time startTime, const double time_inc, std::shared_ptr<tf2_ros::Buffer> tf_);
    //从tf缓存数据中，寻找对应时间戳的里程计位姿
    bool getLaserPose(geometry_msgs::PoseStamped &odom_pose,ros::Time dt,std::shared_ptr<tf2_ros::Buffer> tf_);
    //根据传入参数，对任意一个分段进行插值
    void lidarMotionCalibration(geometry_msgs::PoseStamped frame_base_pose,geometry_msgs::PoseStamped frame_start_pose,geometry_msgs::PoseStamped frame_end_pose,
                                 std::vector<double>& ranges,std::vector<double>& angles,
                                 int startIndex, int& beam_number);
    void publishPointcloud(const std::vector<double> ranges_, const std::vector<double> angles_, const sensor_msgs::LaserScanConstPtr& scan_msg);
//使用点云将激光可视化
#if debug_
    void visual_cloud_scan(const std::vector<double> ranges_, const std::vector<double> angles_,unsigned char r_,unsigned char g_,unsigned char b_);
#endif 
public:
    //声明TF的聆听者、ROS句柄、scan的订阅者
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    // ros::Subscriber scan_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_;
    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;
    ros::Publisher pointcloud_pub_;
    //针对各自的情况需要更改的名字，自行更改
    const std::string scan_frame_name_="laser_link";
    const std::string odom_name_="odom";
    const std::string scan_sub_name_="scan";

    bool scan_time_from_start_;

#if debug_
    //可视化点云对象
    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
#endif
};
//构造函数
LidarMotionCalibrator::LidarMotionCalibrator():private_nh_("~")
{
    private_nh_.param("scan_time_from_start", scan_time_from_start_, true);
    tf_.reset(new tf2_ros::Buffer());
    tfl_.reset(new tf2_ros::TransformListener(*tf_));
    // scan_sub_ = nh_.subscribe(scan_sub_name_, 10, &LidarMotionCalibrator::ScanCallBack, this);
    laser_scan_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 100));
    laser_scan_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                                                *tf_,
                                                                                odom_name_,
                                                                                100,
                                                                                nh_));
    laser_scan_filter_->registerCallback(boost::bind(&LidarMotionCalibrator::ScanCallBack, this, _1));

    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("undistortion_pointcloud", 2, true);
}
//析构函数
LidarMotionCalibrator::~LidarMotionCalibrator()
{
}
//scan回调函数
void LidarMotionCalibrator::ScanCallBack(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
    ros::Time startTime, endTime;
    int beamNum = scan_msg->ranges.size();
    
    // if(scan_time_from_start_)
    // {
    //     //一帧scan的时间戳就代表一帧数据的开始时间
    //     startTime = scan_msg->header.stamp;
    //     // sensor_msgs::LaserScan laserScanMsg = *scan_msg;
    //     // int beamNum = laserScanMsg.ranges.size();
    //     //根据激光时间分割和激光束个数的乘积+startTime得到endTime（最后一束激光束的时间）
    //     endTime = startTime + ros::Duration(scan_msg->time_increment * beamNum);
    // }else{
    //     //一帧scan的时间戳就代表一帧数据的结束时间
    //     endTime   = scan_msg->header.stamp;
    //     //最后一束激光的时间减去扫描时间得到第一束激光的时间
    //     startTime = endTime - ros::Duration(scan_msg->time_increment * beamNum);
    // }
    
    //拷贝数据到angles,ranges
    std::vector<double> angles,ranges;
    for(int i = 0; i < beamNum;i++)
    {
        double lidar_dist = scan_msg->ranges[i];//单位米
        double lidar_angle = scan_msg->angle_min + scan_msg->angle_increment * i;//单位弧度
        ranges.push_back(lidar_dist);
        angles.push_back(lidar_angle);
    }

#if debug_
    visual_cloud_.clear();
    //数据矫正前、封装激光点云可视化、红色
    visual_cloud_scan(ranges,angles,255,0,0);
#endif

    ros::Time start_time;
    ros::Time end_time;
    if(scan_time_from_start_)
    {
        start_time = scan_msg->header.stamp;
        // end_time = laser_scan->header.stamp + ros::Duration(laser_scan->scan_time);
    }else
    {
        start_time = scan_msg->header.stamp - ros::Duration(scan_msg->scan_time);
        // end_time = laser_scan->header.stamp;
    }
    //激光雷达运动畸变去除函数
    lidarCalibration(ranges, angles, start_time, scan_msg->time_increment, tf_);

#if debug_
    //数据矫正后、封装打算点云可视化、绿色
    visual_cloud_scan(ranges,angles,0,255,0);
    g_PointCloudView.showCloud(visual_cloud_.makeShared());
#endif
    publishPointcloud(ranges, angles, scan_msg);
}
//激光雷达运动畸变去除函数
void LidarMotionCalibrator::lidarCalibration(std::vector<double>& ranges,std::vector<double>& angles,ros::Time startTime, const double time_inc, std::shared_ptr<tf2_ros::Buffer> tf_)
{
    //激光束的数量
    int beamNumber = ranges.size();
    //分段时间间隔，单位us
    int interpolation_time_duration = 5 * 1000;//单位us

    // tf::Stamped<tf::Pose> frame_base_pose; //基准坐标系原点位姿
    geometry_msgs::PoseStamped frame_base_pose; //基准坐标系原点位姿
    geometry_msgs::PoseStamped frame_start_pose;
    geometry_msgs::PoseStamped frame_mid_pose;

    double start_time = startTime.toSec() * 1000 * 1000;      //*1000*1000转化时间单位为us
    // double end_time   = endTime.toSec() * 1000 * 1000;
    // double time_inc   = (end_time - start_time) / beamNumber; //每相邻两束激光数据的时间间隔，单位us
    double time_inc_mul = time_inc * 1000000;

    //得到start_time时刻，laser_link在里程计坐标下的位姿，存放到frame_start_pose
    if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
    {
        ROS_WARN("Not Start Pose,Can not Calib");
        return ;
    }
    //分段个数计数
    int cnt = 0;
    //当前插值的段的起始坐标
    int start_index = 0;
    //默认基准坐标系就是第一个位姿的坐标系
    frame_base_pose = frame_start_pose;

    for(int i = 0; i < beamNumber; i++)
    {
        // ROS_INFO("deal with laser msg");
        //按照分割时间分段，分割时间大小为interpolation_time_duration
        double mid_time = start_time + time_inc_mul * (i - start_index);
        //这里的mid_time、start_time多次重复利用
        if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
        {
            cnt++;
            //得到临时结束点的laser_link在里程计坐标系下的位姿，存放到frame_mid_pose
            if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
            {
                ROS_ERROR("Mid %d Pose Error",cnt);
                return ;
            }
            //计算该分段需要插值的个数
            int interp_count = i + 1 - start_index ; 
            //对本分段的激光点进行运动畸变的去除
            lidarMotionCalibration(frame_base_pose,  //对于一帧激光雷达数据，传入参数基准坐标系是不变的
                                    frame_start_pose, //每一次的传入，都代表新分段的开始位姿，第一个分段，根据时间戳，在tf树上获得，其他分段都为上一段的结束点传递
                                    frame_mid_pose,   //每一次的传入，都代表新分段的结束位姿，根据时间戳，在tf树上获得
                                    ranges,           //引用对象，需要被修改的距离数组
                                    angles,           //引用对象，需要被修改的角度数组
                                    start_index,      //每一次的传入，都代表新分段的开始序号
                                    interp_count);    //每一次的传入，都代表该新分段需要线性插值的个数
            //更新时间
            start_time = mid_time;
            start_index = i;   
            ROS_INFO("start_index = %d", start_index);  
            frame_start_pose = frame_mid_pose;        //将上一分段的结束位姿，传递为下一分段的开始位姿
            
            double yaw = tf2::getYaw(frame_mid_pose.pose.orientation);
            ROS_INFO("current index pose info: x = %f, y = %f, yaw = %f", frame_mid_pose.pose.position.x, frame_mid_pose.pose.position.y, yaw);
            
        }
    }

    std::cout << "ranges size = " << ranges.size() << std::endl;
}

//从tf缓存数据中，寻找laser_link对应时间戳的里程计位姿
bool LidarMotionCalibrator::getLaserPose(geometry_msgs::PoseStamped &odom_pose,ros::Time dt,std::shared_ptr<tf2_ros::Buffer> tf_)
{
    // odom_pose.header.frame_id = scan_frame_name_;
    geometry_msgs::PoseStamped ident;
    ident.header.frame_id = scan_frame_name_;
    ident.header.stamp = dt;
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
    //定义一个 tf::Stamped 对象，构造函数的入口参数（const T& input, const ros::Time& timestamp, const std::string & frame_id）
    // tf::Stamped <tf::Pose> tmp(odom_pose, dt, scan_frame_name_);
    try
    {
        //阻塞直到可能进行转换或超时，解决时间不同步问题
        // if(!tf_->lookupTransform(odom_name_, scan_frame_name_, dt)             // 0.15s 的时间可以修改
        // {
        //     ROS_ERROR("LidarMotion-Can not Wait Transform()");
        //     return false;
        // }
        // //转换一个带时间戳的位姿到目标坐标系odom_name_的位姿输出到odom_pose
        // tf_->transformPose(odom_name_, tmp, odom_pose);
        double delta_time = (ros::Time::now() - dt).toSec();
        ROS_INFO("delta_time = %f", delta_time);

        tf_->transform(ident, odom_pose, odom_name_, ros::Duration(0.1));
        
    }
    catch(const tf2::TransformException & e)
    {
        std::cerr << e.what() << '\n';
    }

    return true;
}

//根据传入参数，对任意一个分段进行插值
void LidarMotionCalibrator::lidarMotionCalibration(geometry_msgs::PoseStamped frame_base_pose,geometry_msgs::PoseStamped frame_start_pose,geometry_msgs::PoseStamped frame_end_pose,
                                                    std::vector<double>& ranges,std::vector<double>& angles,
                                                    int startIndex,int& beam_number)
{
    ROS_INFO("-----");
    //beam_step插值函数所用的步长
    double beam_step = 1.0 / (beam_number-1);
    //该分段中，在里程计坐标系下，laser_link位姿的起始角度 和 结束角度，四元数表示
    tf2::Quaternion start_angle_q;
    tf2::convert(frame_start_pose.pose.orientation, start_angle_q);

    tf2::Quaternion end_angle_q;
    tf2::convert(frame_start_pose.pose.orientation, end_angle_q);
    //该分段中，在里程计坐标系下，laser_link位姿的起始角度、该帧激光数据在里程计坐标系下基准坐标系位姿的角度，弧度表示
    double  start_angle_r = tf2::getYaw(start_angle_q);

    tf2::Quaternion frame_base_pose_q;
    tf2::convert(frame_base_pose.pose.orientation, frame_base_pose_q);
    double   base_angle_r = tf2::getYaw(frame_base_pose_q);
    //该分段中，在里程计坐标系下，laser_link位姿的起始位姿、结束位姿，以及该帧激光数据在里程计坐标系下基准坐标系的位姿
    tf2::Vector3 start_pos(frame_start_pose.pose.position.x, frame_start_pose.pose.position.y, frame_start_pose.pose.position.z); start_pos.setZ(0);
    tf2::Vector3   end_pos(frame_end_pose.pose.position.x, frame_end_pose.pose.position.y, frame_end_pose.pose.position.z);   end_pos.setZ(0);   
    tf::Vector3  base_pos(frame_base_pose.pose.position.x, frame_base_pose.pose.position.y, frame_base_pose.pose.position.z);  base_pos.setZ(0);
    //临时变量
    double mid_angle;
    tf2::Vector3 mid_pos;
    tf2::Vector3 mid_point;
    double lidar_angle, lidar_dist;

    //beam_number为该分段中需要插值的激光束的个数
    for(int i = 0; i< beam_number;i++)
    {
        //得到第i个激光束的角度插值，线性插值需要步长、起始和结束数据,与该激光点坐标系和里程计坐标系的夹角
        mid_angle =  tf2::getYaw(start_angle_q.slerp(end_angle_q, beam_step * i));  //slerp（）角度线性插值函数
        //得到第i个激光束的近似的里程计位姿线性插值
        mid_pos = start_pos.lerp(end_pos, beam_step * i);  //lerp（），位姿线性插值函数
        //如果激光束距离不等于无穷,则需要进行矫正
        if( std::isinf(ranges[startIndex + i]) == false)
        {
            //取出该分段中该束激光距离和角度
            lidar_dist  =  ranges[startIndex+i];
            lidar_angle =  angles[startIndex+i];
            //在当前帧的激光雷达坐标系下，该激光点的坐标 （真实的、实际存在的、但不知道具体数值）
            double laser_x,laser_y;
            laser_x = lidar_dist * cos(lidar_angle);
            laser_y = lidar_dist * sin(lidar_angle);
            //该分段中的该激光点，变换的里程计坐标系下的坐标，（这里用插值的激光雷达坐标系近似上面哪个真实存在的激光雷达坐标系，因为知道数值，可以进行计算）
            double  odom_x,odom_y;
            double  cos_ , sin_;
            cos_ = cos(mid_angle);
            sin_ = sin(mid_angle);

            odom_x = laser_x * cos_ - laser_y * sin_ + mid_pos.x();
            odom_y = laser_x * sin_ + laser_y * cos_ + mid_pos.y();
            mid_point.setValue(odom_x, odom_y, 0);
            //在里程计坐标系下，基准坐标系的参数
            double x0,y0,a0,s,c;
            x0 = base_pos.x();
            y0 = base_pos.y();
            a0 = base_angle_r;
            s  = sin(a0);
            c  = cos(a0);
           //把该激光点都从里程计坐标系下，变换的基准坐标系下
            double tmp_x,tmp_y;
            tmp_x =  mid_point.x()*c  + mid_point.y()*s  - x0*c - y0*s;
            tmp_y = -mid_point.x()*s  + mid_point.y()*c  + x0*s - y0*c;
            mid_point.setValue(tmp_x,tmp_y,0);
            //然后计算该激光点以起始坐标为起点的 dist angle
            double dx,dy;
            dx = (mid_point.x());
            dy = (mid_point.y());
            lidar_dist = sqrt(dx*dx + dy*dy);
            lidar_angle = atan2(dy,dx);
            //激光雷达被矫正
            ranges[startIndex+i] = lidar_dist;
            angles[startIndex+i] = lidar_angle;
        }
        //如果等于无穷,则随便计算一下角度
        else
        {
            double tmp_angle;            
            lidar_angle = angles[startIndex+i];
            //里程计坐标系的角度
            tmp_angle = mid_angle + lidar_angle;
            tmp_angle = tfNormalizeAngle(tmp_angle);
            //如果数据非法 则只需要设置角度就可以了。把角度换算成start_pos坐标系内的角度
            lidar_angle = tfNormalizeAngle(tmp_angle - start_angle_r);
            angles[startIndex+i] = lidar_angle;
        }
    }

    ROS_INFO("****");
}

//使用点云将激光可视化
#if debug_
void LidarMotionCalibrator::visual_cloud_scan(std::vector<double> ranges_,const std::vector<double> angles_,unsigned char r_,unsigned char g_,unsigned char b_)
{
    unsigned char r = r_, g = g_, b = b_; //变量不要重名 
    for(int i = 0; i < ranges_.size();i++)
    {
        if(ranges_[i] < 0.05 || std::isnan(ranges_[i]) || std::isinf(ranges_[i]))
            continue;

        pcl::PointXYZRGB pt;
        pt.x = ranges_[i] * cos(angles_[i]);
        pt.y = ranges_[i] * sin(angles_[i]);
        pt.z = 1.0;

        // pack r/g/b into rgb
        unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
        pt.rgb = *reinterpret_cast<float*>(&rgb);
        
        visual_cloud_.push_back(pt);
    }     

}
#endif 

void LidarMotionCalibrator::publishPointcloud(const std::vector<double> ranges_, const std::vector<double> angles_, const sensor_msgs::LaserScanConstPtr& scan_msg)
{
    static auto laser_in_base = tf_->lookupTransform("base_link", scan_msg->header.frame_id, ros::Time(0));
    static float x = laser_in_base.transform.translation.x;
    static float y = laser_in_base.transform.translation.y;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.width = ranges_.size();
    pcl_cloud.height = 1;
    pcl_cloud.resize(ranges_.size());
    sensor_msgs::PointCloud2 filter_cloud;

    for (int i = 0; i < ranges_.size(); ++i) {
        pcl_cloud.points[i].x = ranges_[i] * cos(angles_[i]);;
        pcl_cloud.points[i].y = ranges_[i] * sin(angles_[i]);
        pcl_cloud.points[i].z = 0;
    }

    pcl::toROSMsg(pcl_cloud, filter_cloud);
    filter_cloud.header.frame_id = "laser_link";
    filter_cloud.header.stamp = scan_msg->header.stamp;
    pointcloud_pub_.publish(filter_cloud);
    ROS_INFO("filter_cloud size = %d", filter_cloud.data.size());
}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"LidarMotionCalib");

    LidarMotionCalibrator tmpLidarMotionCalib;//运动畸变去除的对象

    ros::spin();
    return 0;
}
