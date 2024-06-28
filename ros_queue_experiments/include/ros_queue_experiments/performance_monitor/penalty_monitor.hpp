#pragma once

#include "ros/ros.h"
#include "ros_queue_experiments/AuvStates.h"
#include "ros_queue_msgs/QueueServerStatsFetch.h"

class PenaltyMonitor
{
    public:
        /**
         * @brief Initialized the penalty monitor by instantiating the publisher for the performance metric
         * and initializing the time reference.
        */
        PenaltyMonitor(ros::NodeHandle& nh);

        /**
         * @brief Method called to compute real penalties and the penalties experienced by the controller.
         * @param msg Message containing the real state metric.
         * @param queue_stats_metric_srv Message containing the estimated metrics from the queue server.
         */
        void realStateMetricCallback(const ros_queue_experiments::AuvStates::ConstPtr& msg,
                                    const ros_queue_msgs::QueueServerStatsFetch& queue_stats_metric_srv);   
    private:
        /**
         * @brief private node handle
        */
        ros::NodeHandle pnh_;

        /**
         * @brief The number of samples to compute the mean of the penalty 
        */
        long long nb_samples_penalty_ = 0;

        /**
         * @brief The cumulative sum of all the incoming real penalties.
        */
        double real_penalty_sum_ = 0.0;

        /**
         * @brief The cumulative sum of all the incoming penalties that thought by the controller.
        */
        double controller_penalty_sum_ = 0.0;

        /**
         * @brief Time reference to compute the time average of the penalty.
        */
        ros::Time init_time_;

        /**
         * @brief Publisher to expose all the stats behind the penalty metric.
        */
        ros::Publisher performance_metric_pub_;
};