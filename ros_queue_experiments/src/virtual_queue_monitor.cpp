#include "ros_queue_experiments/performance_monitor/virtual_queue_monitor.hpp"

#include "std_msgs/Empty.h"
#include "ros_queue_msgs/QueueServerStatsFetch.h"

VirtualQueueMonitor::VirtualQueueMonitor(ros::NodeHandle& nh): nhp_(nh), nh_(ros::NodeHandle()), localization_monitor_(nhp_), temperature_monitor_(nhp_)
{
    queue_server_stats_client_ = nh_.serviceClient<ros_queue_msgs::QueueServerStatsFetch>("queue_server/get_server_stats");
    ROS_INFO_STREAM("Waiting for existence of the "<< queue_server_stats_client_.getService() << " service");
    queue_server_stats_client_.waitForExistence();
    ROS_INFO_STREAM("Service "<< queue_server_stats_client_.getService() << " exists");

    monitoring_sample_done_pub_ = nhp_.advertise<std_msgs::Empty>("monitoring_done",1);
    real_state_sub_ = nh_.subscribe("auv_system_node/auv_state", 1, &VirtualQueueMonitor::realStateCallback, this);
}

void VirtualQueueMonitor::realStateCallback(const ros_queue_experiments::AuvStates::ConstPtr& msg)
{
    ros_queue_msgs::QueueServerStatsFetch queue_stats_metric_srv;
    
    if(queue_server_stats_client_.call(queue_stats_metric_srv))
    {
        localization_monitor_.realStateMetricCallback(msg, queue_stats_metric_srv);
        temperature_monitor_.realStateMetricCallback(msg, queue_stats_metric_srv);
        
        monitoring_sample_done_pub_.publish(std_msgs::Empty());
    }
    else
    {
        ROS_ERROR_STREAM("Virtual queue monitoring node failed to call service "<< queue_server_stats_client_.getService());
    }
}