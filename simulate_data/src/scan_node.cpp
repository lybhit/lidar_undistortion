#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class ScanNode
{
public:
    ScanNode();
    ~ScanNode();
    
    void publishScanMsg(const ros::TimerEvent& te);
    void run();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher scan_pub_;  

    int laser_frequency_;
    int num_readings_;
    double range_;

    ros::Timer updateTimer_;

};

ScanNode::ScanNode(): private_nh_("~")
{
    private_nh_.param("laser_frequency", laser_frequency_, 50);
    private_nh_.param("num_readings", num_readings_, 100);
    private_nh_.param("range", range_, 50.0);

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 50);
}

ScanNode::~ScanNode()
{

}

void ScanNode::publishScanMsg(const ros::TimerEvent& te)
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
    scan.header.frame_id = "base_link";
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

void ScanNode::run(void)
{
    // The timer ensures periodic data publishing
    updateTimer_ = ros::Timer(nh_.createTimer(ros::Duration(1/laser_frequency_),
                                            &ScanNode::publishScanMsg,
                                            this));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_publisher");

    ScanNode scan_node;
    scan_node.run();

    ros::spin();
    return 0;
}
