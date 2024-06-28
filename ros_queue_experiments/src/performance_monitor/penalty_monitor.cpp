#include "ros_queue_experiments/performance_monitor/penalty_monitor.hpp"

#include "ros_queue_experiments/MetricPerformance.h"

PenaltyMonitor::PenaltyMonitor(ros::NodeHandle& nh): pnh_(nh)
{
    performance_metric_pub_ = pnh_.advertise<ros_queue_experiments::MetricPerformance>("penalty", 1);
    init_time_ = ros::Time::now();
}

void PenaltyMonitor::realStateMetricCallback(const ros_queue_experiments::AuvStates::ConstPtr& msg,
                                            const ros_queue_msgs::QueueServerStatsFetch& queue_stats_metric_srv)
{
    const double real_penalty = msg->real_penalty;
    const double controller_penalty = msg->controller_penalty;
    const float current_elapsed_time = (ros::Time::now() - init_time_).toSec();

    real_penalty_sum_ += real_penalty;
    controller_penalty_sum_ += controller_penalty;
    nb_samples_penalty_++;

    ros_queue_experiments::MetricPerformance metric_performance_msg;
    metric_performance_msg.header.stamp = ros::Time::now();
    metric_performance_msg.metric_name = "penalty";

    metric_performance_msg.current_real_value = real_penalty;
    metric_performance_msg.real_average_value = real_penalty_sum_ / nb_samples_penalty_;
    metric_performance_msg.real_time_average_value = real_penalty_sum_/current_elapsed_time;

    metric_performance_msg.target_value = msg->controller_penalty;  // This one is misleading but it's for the sake of usage of the same message
    metric_performance_msg.queue_server_arrival_mean = controller_penalty_sum_ / nb_samples_penalty_;
    metric_performance_msg.queue_server_time_average_value = controller_penalty_sum_/current_elapsed_time;

    metric_performance_msg.real_continuous_average_diff_with_server_mean = metric_performance_msg.real_average_value - metric_performance_msg.queue_server_arrival_mean;
    performance_metric_pub_.publish(metric_performance_msg);
}
