#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ros_queue_experiments/ActionPerformance.h"

#include "ros_queue_experiments/auv_states.hpp"

void action_messages_callbacks(const ros_queue_experiments::ActionPerformance::ConstPtr& perturbated_action_performance, 
                               const ros_queue_experiments::ActionPerformance::ConstPtr& optimal_action_performance)
{

    ros_queue_experiments::ActionPerformance synchronized_action_performance;
    synchronized_action_performance.header.stamp = ros::Time::now();

    synchronized_action_performance.target_action = perturbated_action_performance->target_action;
    synchronized_action_performance.applied_action = perturbated_action_performance->applied_action;
    synchronized_action_performance.optimal_action = optimal_action_performance->target_action;

    synchronized_action_performance.action_index_difference = AUVStates::getZoneFromTransmissionVector(synchronized_action_performance.applied_action) - AUVStates::getZoneFromTransmissionVector(synchronized_action_performance.target_action);
    synchronized_action_performance.optimal_action_index_difference = AUVStates::getZoneFromTransmissionVector(synchronized_action_performance.optimal_action) - AUVStates::getZoneFromTransmissionVector(synchronized_action_performance.target_action);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "synced_action_performance_monitor");

    ros::NodeHandle nh;

    ros::Publisher synchronized_action_performance_publisher = nh.advertise<ros_queue_experiments::ActionPerformance>("synced_action_performances", 10);

    message_filters::Subscriber<ros_queue_experiments::ActionPerformance> perturbated_action_peformance_sub(nh, "/perturbation_node/action_performance", 1);
    message_filters::Subscriber<ros_queue_experiments::ActionPerformance> optimal_action_performance_sub(nh, "/optimal_action_node/action_performance", 1);

    typedef message_filters::sync_policies::ApproximateTime<ros_queue_experiments::ActionPerformance, ros_queue_experiments::ActionPerformance> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), perturbated_action_peformance_sub, optimal_action_performance_sub);
    sync.registerCallback(boost::bind(&action_messages_callbacks, _1, _2));

    ros::spin();

    return 0;
}