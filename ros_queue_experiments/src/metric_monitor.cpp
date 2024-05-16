#include "ros_queue_experiments/performance_monitor/metric_monitor.hpp"
#include "ros_queue_msgs/QueueServerStatsFetch.h"
#include "ros_queue_experiments/MetricPerformance.h"

MetricMonitor::MetricMonitor(ros::NodeHandle& nh): nh_(nh), ns_nh_(ros::NodeHandle())
{
    queue_stats_metric_client_ = ns_nh_.serviceClient<ros_queue_msgs::QueueServerStatsFetch>("queue_server/get_server_stats");
    ROS_INFO_STREAM("Waiting for existence of the "<< queue_stats_metric_client_.getService() << " service");
    queue_stats_metric_client_.waitForExistence();
    ROS_INFO_STREAM("Service "<< queue_stats_metric_client_.getService() << " exists");

    real_state_metric_sub_ = ns_nh_.subscribe("auv_system_node/auv_state", 1, &MetricMonitor::realStateMetricCallback, this);
    real_state_metric_sum_ = 0;
    nb_real_state_samples_ = 0;
    init_time_ = ros::Time::now();
}

void MetricMonitor::realStateMetricCallback(const ros_queue_experiments::AuvStates::ConstPtr& msg)
{
    ros_queue_experiments::MetricPerformance metric_performance_msgs;

    ros_queue_msgs::QueueServerStatsFetch queue_stats_metric_srv_;
    
    if(queue_stats_metric_client_.call(queue_stats_metric_srv_))
    {
        metric_performance_msgs.header.stamp = ros::Time::now();

        metric_performance_msgs.metric_name = metric_name_;

        double real_state_metric_value = getMetricFromAuvState(msg);
        metric_performance_msgs.current_real_value = real_state_metric_value;

        addNewRealStateMetricSample(real_state_metric_value);
        metric_performance_msgs.real_average_value = computeRealStateMetricMean();
        metric_performance_msgs.real_time_average_value = computeRealStateMetricTimeAverage();
        metric_performance_msgs.real_continous_average_value = computeRealStateMetricContinuousMean(getContinuousIntegralOfMetricFromAuvState(msg));

        metric_performance_msgs.queue_server_time_average_value = getQueueServerTimeAverageMetric(queue_stats_metric_srv_.response);

        double target  = getMetricTarget();
        metric_performance_msgs.target_value = target;
        metric_performance_msgs.real_current_diff_with_target = real_state_metric_value - target;
        metric_performance_msgs.real_mean_diff_with_target = metric_performance_msgs.real_average_value - target;
        metric_performance_msgs.real_time_average_diff_with_server_time_average = metric_performance_msgs.real_time_average_value - metric_performance_msgs.queue_server_time_average_value;

        metric_performance_msgs.seconds_since_start = (ros::Time::now() - init_time_).toSec();
        performance_metric_pub_.publish(metric_performance_msgs);
    }
    else
    {
        ROS_ERROR("Failed to call service queue_server/get_server_stats");
    }
}

void MetricMonitor::addNewRealStateMetricSample(double new_metric_value_sample)
{
    real_state_metric_sum_ += new_metric_value_sample;
    nb_real_state_samples_++;
}

double MetricMonitor::computeRealStateMetricMean()
{
    if(nb_real_state_samples_ == 0)
    {
        return 0;
    }

    return real_state_metric_sum_ / nb_real_state_samples_;
}

double MetricMonitor::computeRealStateMetricTimeAverage()
{
    if(nb_real_state_samples_ == 0)
    {
        return 0;
    }

    return real_state_metric_sum_ / (ros::Time::now() - init_time_).toSec();
}


double MetricMonitor::computeRealStateMetricContinuousMean(double time_integral_of_metric)
{
    if(nb_real_state_samples_ == 0)
    {
        return 0;
    }

    return time_integral_of_metric / (ros::Time::now() - init_time_).toSec();
}