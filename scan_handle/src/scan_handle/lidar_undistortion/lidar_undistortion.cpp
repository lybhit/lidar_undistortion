#include "scan_handle/lidar_undistortion/lidar_undistortion.h"

LidarMotionCalibrator::LidarMotionCalibrator(std::string scan_frame_name, std::string base_frame_name, std::string odom_frame_name)
{
    scan_frame_name_ = scan_frame_name;
    base_frame_name_ = base_frame_name;
    odom_frame_name_ = odom_frame_name;

    tf_.reset(new tf2_ros::Buffer(ros::Duration(10)));
    tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

LidarMotionCalibrator::~LidarMotionCalibrator()
{}

//激光雷达运动畸变去除函数
void LidarMotionCalibrator::lidarCalibration(std::vector<double>& ranges, std::vector<double>& angles,ros::Time startTime, const ros::Time endTime, const double time_inc)
{
    // ROS_INFO("startTime = %f, endTime = %f, time_inc = %f", startTime.toSec(), endTime.toSec(), time_inc);
    //激光束的数量
    int beamNumber = ranges.size();
    //分段时间间隔，单位us
    int interpolation_time_duration = 5 * 1000;//单位us

    geometry_msgs::PoseStamped frame_target_pose; //基准坐标系原点位姿
    geometry_msgs::PoseStamped frame_start_pose;  //激光起始点位姿
    geometry_msgs::PoseStamped frame_source_pose; //激光当前投影点位姿

    double start_time = startTime.toSec() * 1000 * 1000;      //*1000*1000转化时间单位为us
    // double end_time   = endTime.toSec() * 1000 * 1000;
    // double time_inc   = (end_time - start_time) / beamNumber; //每相邻两束激光数据的时间间隔，单位us
    double time_inc_mul = time_inc * 1000000;

    //得到start_time时刻，laser_link在里程计坐标下的位姿，存放到frame_start_pose
    if(getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0)))
    {
        ROS_INFO("start_pose: x = %f, y = %f", frame_start_pose.pose.position.x, frame_start_pose.pose.position.y);
    }else{
        ROS_INFO("Not Start Pose, Can not Calib");
        return ;
    }

    //直接获取最后一束激光作为基准坐标原点位姿
    if(getLaserPose(frame_target_pose, endTime))
    {
        ROS_INFO("end_pose: x = %f, y = %f", frame_target_pose.pose.position.x, frame_target_pose.pose.position.y);
    }else{
        ROS_INFO("Not end Pose, Can not Calib");
        return ;
    }
    //分段个数计数
    int cnt = 0;
    //当前插值的段的起始坐标
    int start_index = 0;

    for(int i = 0; i < beamNumber; i++)
    {
        double mid_time = start_time + time_inc_mul * i;
        cnt++;
        //得到临时结束点的laser_link在里程计坐标系下的位姿，存放到frame_source_pose
        if(!getLaserPose(frame_source_pose, ros::Time(mid_time/1000000.0)))
        {
            ROS_ERROR("Mid %d Pose Error",cnt);
            return ;
        }
        //计算该分段需要插值的个数
        int interp_count = i - start_index; 
        //对本分段的激光点进行运动畸变的去除
        lidarMotionCalibration(frame_target_pose, //对于一帧激光雷达数据，传入参数基准坐标系是不变的 
                                frame_source_pose,//每一次的传入，当前处理激光数据的位姿，在tf树上获得
                                ranges,           //引用对象，需要被修改的距离数组
                                angles,           //引用对象，需要被修改的角度数组
                                i);               //每次处理的激光数据的索引
    }

    // std::cout << "ranges size = " << ranges.size() << std::endl;
}

//根据传入参数，对任意一个分段进行插值
void LidarMotionCalibrator::lidarMotionCalibration(geometry_msgs::PoseStamped frame_target_pose, geometry_msgs::PoseStamped frame_source_pose, std::vector<double>& ranges,std::vector<double>& angles, int startIndex)
{
    tf2::Quaternion end_angle_q;
    tf2::convert(frame_source_pose.pose.orientation, end_angle_q);
    //该分段中，在里程计坐标系下，laser_link位姿的起始角度、该帧激光数据在里程计坐标系下基准坐标系位姿的角度，弧度表示
    double start_angle_r = tf2::getYaw(end_angle_q);

    tf2::Quaternion frame_target_pose_q;
    tf2::convert(frame_target_pose.pose.orientation, frame_target_pose_q);
    double base_angle_r = tf2::getYaw(frame_target_pose_q);

    tf2::Vector3  end_pos(frame_source_pose.pose.position.x, frame_source_pose.pose.position.y, frame_source_pose.pose.position.z);   end_pos.setZ(0);   
    tf::Vector3  base_pos(frame_target_pose.pose.position.x, frame_target_pose.pose.position.y, frame_target_pose.pose.position.z);  base_pos.setZ(0);
    //临时变量
    double mid_angle;
    tf2::Vector3 mid_pos;
    tf2::Vector3 mid_point;
    double lidar_angle, lidar_dist;

    {
        mid_angle = start_angle_r;
        mid_pos = end_pos;
        //如果激光束距离不等于无穷,则需要进行矫正
        if( std::isinf(ranges[startIndex]) == false)
        {
            //取出该分段中该束激光距离和角度
            lidar_dist  =  ranges[startIndex];
            lidar_angle =  angles[startIndex];
            // ROS_INFO_STREAM_COND((startIndex+i)%100 == 0, "process index = " << startIndex+i << " ,current range = " << lidar_dist << " ,current angle = " << lidar_angle);
            //在当前帧的激光雷达坐标系下，该激光点的坐标 （真实的、实际存在的、但不知道具体数值）
            double laser_x,laser_y;
            laser_x = lidar_dist * cos(lidar_angle);
            laser_y = lidar_dist * sin(lidar_angle);
            //该分段中的该激光点，变换的里程计坐标系下的坐标，（这里用插值的激光雷达坐标系近似上面哪个真实存在的激光雷达坐标系，因为知道数值，可以进行计算）
            double  odom_x,odom_y;
            double  cos_ , sin_;
            cos_ = cos(mid_angle);
            sin_ = sin(mid_angle);
            // ROS_INFO_STREAM_COND(i % 100 == 0,"undistortion beam = " << i << " interplate angle = " << mid_angle);

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
            ranges[startIndex] = lidar_dist;
            angles[startIndex] = lidar_angle;
        }
        else
        {
            /*
              Todo something useful. 
            */
        }
    }
}

//从tf缓存数据中，寻找laser_link对应时间戳的里程计位姿
bool LidarMotionCalibrator::getLaserPose(geometry_msgs::PoseStamped &odom_pose, ros::Time dt)
{
    geometry_msgs::TransformStamped transformStamped_laser;
    geometry_msgs::TransformStamped transformStamped_base;

    try{
      transformStamped_laser = this->tf_->lookupTransform(odom_frame_name_, scan_frame_name_, dt, ros::Duration(0.05));
      transformStamped_base  = this->tf_->lookupTransform(odom_frame_name_, base_frame_name_, dt, ros::Duration(0.05));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }

    odom_pose.pose.position.x = transformStamped_laser.transform.translation.x;
    odom_pose.pose.position.y = transformStamped_laser.transform.translation.y;
    odom_pose.pose.position.z = transformStamped_laser.transform.translation.z;
    odom_pose.pose.orientation = transformStamped_base.transform.rotation;

    return true;
}
