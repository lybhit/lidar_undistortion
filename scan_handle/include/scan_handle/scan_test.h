#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"
#include "config.h"
#include "scan_handle/scan_data_handle/scan_manager.h"

#include <map>

class ScanTest{
    public:
        ScanTest();
        ~ScanTest();

    private:
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        std::shared_ptr<tf2_ros::TransformListener> tfl_;
        std::shared_ptr<tf2_ros::Buffer> tf_;

        void laserReceived(const sensor_msgs::LaserScanConstPtr& );
        void odomReceived(const nav_msgs::Odometry& odom_msg);

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher scan_cloud_pub_;
        ros::Subscriber odom_sub_;
        
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_;
        std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;

        std::map<std::string, int> frame_to_laser_;
        Config config_;
        ScanManager scan_manager_;

};
