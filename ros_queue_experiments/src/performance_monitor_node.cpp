#include "ros/ros.h"

#include "ros_queue_experiments/performance_monitor/localization_monitor.hpp"
#include "ros_queue_experiments/performance_monitor/real_queue_monitor.hpp"
#include "ros_queue_experiments/performance_monitor/temperature_monitor.hpp"
#include "ros_queue_experiments/performance_monitor/action_monitor.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "performance_monitor_node");
    ros::NodeHandle nh("~");

    LocalizationMonitor localization_monitor(nh);
    TemperatureMonitor temperature_monitor(nh);
    RealQueueMonitor real_queue_monitor(nh);
    ActionMonitor action_monitor(nh, "/perturbation_node/action_performance", "/optimal_action_node/action_performance");

    ros::spin();

    return 0;
}