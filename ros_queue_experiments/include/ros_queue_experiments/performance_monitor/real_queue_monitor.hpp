#pragma once

#include "ros/ros.h"
#include <string>

#include "ros_queue_msgs/QueueServerStats.h"

class RealQueueMonitor
{
    public:
        RealQueueMonitor(ros::NodeHandle& nh);

    protected:
        /**
         * @brief Node handle for the metric monitor.
        */
        ros::NodeHandle nh_;

        /**
         * @brief Name of the queue real queue. Used for logging and to fetch the queue in the queue server
        */
        std::string queue_name_;

        /**
         * @brief Susbcriber the stats of the queue server.
        */
        ros::Subscriber queue_stats_sub_;
        
        /**
         * @brief Metric publisher. Should be defined by child class.
         */
        ros::Publisher performance_metric_pub_;
        
        /**
         * @brief Callback for the queue stats.
        */
        void queueStatsCallback(const ros_queue_msgs::QueueServerStats::ConstPtr& msg);
};