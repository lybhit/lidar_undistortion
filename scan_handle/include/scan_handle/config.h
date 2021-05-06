#ifndef CONFIG_H
#define CONFIG_H

#include <string>

class Config{

    public:
        double laser_min_range = 0.1;
        double laser_max_range = 80;
        bool scan_time_from_start = false;
        std::string base_frame_id = "base_link";
        std::string odom_frame_id = "odom";
        std::string map_frame_id_ = "map";
};

#endif
