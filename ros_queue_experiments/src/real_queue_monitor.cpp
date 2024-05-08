#include "ros_queue_experiments/performance_monitor/real_queue_monitor.hpp"

#include "ros_queue_experiments/MetricPerformance.h"

RealQueueMonitor::RealQueueMonitor(ros::NodeHandle& nh): nh_(nh), ns_nh_(ros::NodeHandle())
{
    queue_name_ = "TaskQueue";
    queue_stats_sub_ = ns_nh_.subscribe("queue_server/server_stats", 1, &RealQueueMonitor::queueStatsCallback, this);
    performance_metric_pub_ = nh_.advertise<ros_queue_experiments::MetricPerformance>("real_queue", 1);
}

void RealQueueMonitor::queueStatsCallback(const ros_queue_msgs::QueueServerStats::ConstPtr& msg)
{
    ros_queue_experiments::MetricPerformance metric_performance_msg;

    for (auto queue_stats : msg->queue_stats)
    {
        if(queue_stats.queue_name == queue_name_)
        {
            metric_performance_msg.metric_name = queue_name_;
            metric_performance_msg.real_average_value = queue_stats.arrival_mean;
            metric_performance_msg.queue_server_time_average_value = queue_stats.arrival_mean;
            metric_performance_msg.target_value = queue_stats.departure_mean;
            metric_performance_msg.real_mean_diff_with_target = queue_stats.arrival_mean - queue_stats.departure_mean;
            performance_metric_pub_.publish(metric_performance_msg);
            return;
        }
    }
    ROS_WARN_THROTTLE(1, "Queue server does not have a queue named %s", queue_name_.c_str());
}