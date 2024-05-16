#pragma once

#include <ros/ros.h>
#include <string>

#include "ros_queue_experiments/AuvStates.h"
#include "ros_queue_msgs/QueueServerStatsFetch.h"

/**
 * @brief Virtual class that monitors a specifif metric from a system and from its estimation given by the queue server.
 * The class publishes the synchronized measurement from the queue server and the systems alongside their difference. 
 * The sampling frequency is based on the publication of the real state metric pulbishing rate.
*/
class MetricMonitor
{
    public:
        MetricMonitor(ros::NodeHandle& nh);

    protected:
        /**
         * @brief Name of the metric. Used for logging.
        */
        std::string metric_name_;

        /**
         * @brief Private node handle for the metric monitor.
         */
        ros::NodeHandle nh_;

        /**
         * @brief Node handle for namespace resolution at the namespace of the node (non-private node handle).  
        */
        ros::NodeHandle ns_nh_;

        /**
         * @brief Metric publisher. Should be defined by child class.
         */
        ros::Publisher performance_metric_pub_;

        /**
         * @brief Service client that gets the metric target.
         */
        ros::ServiceClient metric_target_client_;
        
        /**
         * @brief Get metric from auv state message.
         * @param msg Message containing the auv state.
         * @return Metric value.
         */
        virtual double getMetricFromAuvState(const ros_queue_experiments::AuvStates::ConstPtr& msg) = 0;

        /**
         * @brief Get the theoritical integral of the continuous function of a given metric from the auv state message.
         * @param msg Message containing the auv state.
         * @return Integral of the metric.
        */
        virtual double getContinuousIntegralOfMetricFromAuvState(const ros_queue_experiments::AuvStates::ConstPtr& msg)=0;

        /**
         * @brief Get the time average the estimated metric from the queue server message
         * @param msg Message containing the queue server stats.
         * @return Time average of the metric estimated by the queue server.
         */
        virtual double getQueueServerTimeAverageMetric(const ros_queue_msgs::QueueServerStatsFetch::Response& msg) = 0;

        /**
         * @brief Get the metric target from the auv system.
         */
        virtual double getMetricTarget() = 0;        
    
    private:
        /**
         * @brief Service client that gets the statitstics of the queue server.
         */
        ros::ServiceClient queue_stats_metric_client_;

        /**
         * @brief Callback for the real state metric.
         * @param msg Message containing the real state metric.
         */
        void realStateMetricCallback(const ros_queue_experiments::AuvStates::ConstPtr& msg);

        /**
         * @brief Real state metric subscriber.
         */
        ros::Subscriber real_state_metric_sub_;
        
        /**
         * @brief Compute the mean of the real state metric
         * @return Mean of the real state metric.
         */
        double computeRealStateMetricMean();

        /**
         * @brief Compute the time average of the real state metric.
         * @return Time average of the real state metric.
         */
        double computeRealStateMetricTimeAverage();

        /**
         * @brief Compute the mean from the integral of the theoritical continuous 
         * function of the metric by dividing it by the elapsed time since the start.
         * @param time_integral_of_metric Integral of the metric.
         * @return Mean of the real state metric based on its integral.
        */
        double computeRealStateMetricContinuousMean(double time_integral_of_metric);

        /**
         * @brief Add new sample of the real metric.
         */
        void addNewRealStateMetricSample(double new_metric_value_sample);
        
        /**
         *  @brief Sum of real state metrics. 
         */
        double real_state_metric_sum_;
        
        /**
         * @brief Number of real state samples.
         */
        long long nb_real_state_samples_;
        
        /**
         * @brief Time at initialization.
         */
        ros::Time init_time_;
};