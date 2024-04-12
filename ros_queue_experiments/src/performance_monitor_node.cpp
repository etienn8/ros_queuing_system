#include "ros/ros.h"
#include "ros_queue_experiments/performance_monitor/localization_monitor.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "performance_monitor_node");
    ros::NodeHandle nh("~");

    LocalizationMonitor localization_monitor(nh);

    ros::spin();

    return 0;
}