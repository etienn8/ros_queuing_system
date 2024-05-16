#include "ros_queue_experiments/performance_monitor/localization_monitor.hpp"

#include "ros_queue_msgs/FloatRequest.h"

#include "ros_queue_experiments/MetricPerformance.h"

LocalizationMonitor::LocalizationMonitor(ros::NodeHandle& nh): MetricMonitor(nh)
{
    metric_name_ = "Localization";
    // There could be a race condition since this publisher might not be ready when the first callback of the auv state publisher is called.
    performance_metric_pub_ = nh_.advertise<ros_queue_experiments::MetricPerformance>("localization", 1);
    
    metric_target_client_ = ns_nh_.serviceClient<ros_queue_msgs::FloatRequest>("auv_system_node/localization/departure/rate/real_metric");
    ROS_INFO_STREAM("Waiting for existence of the "<< metric_target_client_.getService() << " service");
    metric_target_client_.waitForExistence();
    ROS_INFO_STREAM("Service "<< metric_target_client_.getService() << " exists");
}

double LocalizationMonitor::getMetricFromAuvState(const ros_queue_experiments::AuvStates::ConstPtr& msg)
{
    return msg->localization;
}

double LocalizationMonitor::getQueueServerTimeAverageMetric(const ros_queue_msgs::QueueServerStatsFetch::Response& msg)
{
    for (auto queue_stats : msg.queue_stats.queue_stats)
    {
        if(queue_stats.queue_name == "LocalizationQueue")
        {
            return queue_stats.arrival_mean;
        }
    }
    ROS_WARN_THROTTLE(1, "Queue server does not have a queue named %s", "LocalizationQueue");
    return 0.0;
}

double LocalizationMonitor::getMetricTarget()
{
    ros_queue_msgs::FloatRequest metric_target_srv;
    if(metric_target_client_.call(metric_target_srv))
    {
        return metric_target_srv.response.value;
    }
    else
    {
        ROS_ERROR("Failed to call service auv_system_node/localization/departure/real_metric");
        return 0.0;
    }
}

double LocalizationMonitor::getContinuousIntegralOfMetricFromAuvState(const ros_queue_experiments::AuvStates::ConstPtr& msg)
{
    return msg->localization_integral;
}