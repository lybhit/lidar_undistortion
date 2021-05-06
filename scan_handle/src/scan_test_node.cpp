#include "scan_handle/scan_test.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_test_node");

    ScanTest scan_test;

    ros::spin();

    return 0;
}