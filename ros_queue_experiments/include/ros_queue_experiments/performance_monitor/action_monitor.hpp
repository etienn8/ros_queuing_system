#pragma once

#include <string>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ros_queue_experiments/ActionPerformance.h"


/**
 * @brief Class that synchronizes the perturbated and optimal action performance messages and publishes the synchronized message.
*/
class ActionMonitor
{
    public:
        ActionMonitor(ros::NodeHandle& nh, const std::string& perturbated_action_topic, const std::string& optimal_action_topic);


    private:
        ros::NodeHandle nh_;

        ros::NodeHandle ns_nh_;

        ros::Publisher synchronized_action_performance_publisher;

        message_filters::Subscriber<ros_queue_experiments::ActionPerformance> perturbated_action_peformance_sub;
        message_filters::Subscriber<ros_queue_experiments::ActionPerformance> optimal_action_performance_sub;

        typedef message_filters::sync_policies::ApproximateTime<ros_queue_experiments::ActionPerformance, ros_queue_experiments::ActionPerformance> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;

        void action_messages_callbacks(const ros_queue_experiments::ActionPerformance::ConstPtr& perturbated_action_performance, 
                                       const ros_queue_experiments::ActionPerformance::ConstPtr& optimal_action_performance);
};