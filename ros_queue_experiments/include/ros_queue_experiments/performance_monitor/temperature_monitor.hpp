#pragma once 

#include "metric_monitor.hpp"
#include "ros_queue_msgs/QueueServerStatsFetch.h"

class TemperatureMonitor: public MetricMonitor
{
    public:
        TemperatureMonitor(ros::NodeHandle& nh);

    protected:
        virtual double getMetricFromAuvState(const ros_queue_experiments::AuvStates::ConstPtr& msg) override;

        virtual double getQueueServerTimeAverageMetric(const ros_queue_msgs::QueueServerStatsFetch::Response& msg)override;
        
        virtual double getMetricTarget() override;

        virtual double getContinuousIntegralOfMetricFromAuvState(const ros_queue_experiments::AuvStates::ConstPtr& msg) override;

        virtual double getQueueServerArrivalMeanMetric(const ros_queue_msgs::QueueServerStatsFetch::Response& msg) override;
};