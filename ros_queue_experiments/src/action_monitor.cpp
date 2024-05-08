#include "ros_queue_experiments/performance_monitor/action_monitor.hpp"
#include "ros_queue_experiments/auv_states.hpp"

ActionMonitor::ActionMonitor(ros::NodeHandle& nh, const std::string& perturbated_action_topic, const std::string& optimal_action_topic):
    nh_(nh),
    ns_nh_(ros::NodeHandle()),
    perturbated_action_peformance_sub(ns_nh_, perturbated_action_topic, 1),
    optimal_action_performance_sub(ns_nh_, optimal_action_topic, 1),
    sync(MySyncPolicy(10), perturbated_action_peformance_sub, optimal_action_performance_sub)
{
    synchronized_action_performance_publisher = nh_.advertise<ros_queue_experiments::ActionPerformance>("synced_action_performances", 10);
    sync.registerCallback(boost::bind(&ActionMonitor::action_messages_callbacks, this, _1, _2));
}

void ActionMonitor::action_messages_callbacks(const ros_queue_experiments::ActionPerformance::ConstPtr& perturbated_action_performance, 
                                              const ros_queue_experiments::ActionPerformance::ConstPtr& optimal_action_performance)
{
        ros_queue_experiments::ActionPerformance synchronized_action_performance;
    synchronized_action_performance.header.stamp = ros::Time::now();

    synchronized_action_performance.target_action = perturbated_action_performance->target_action;
    synchronized_action_performance.applied_action = perturbated_action_performance->applied_action;
    synchronized_action_performance.optimal_action = optimal_action_performance->target_action;

    synchronized_action_performance.action_index_difference = AUVStates::getZoneFromTransmissionVector(synchronized_action_performance.applied_action) - AUVStates::getZoneFromTransmissionVector(synchronized_action_performance.target_action);
    synchronized_action_performance.optimal_action_index_difference = AUVStates::getZoneFromTransmissionVector(synchronized_action_performance.optimal_action) - AUVStates::getZoneFromTransmissionVector(synchronized_action_performance.target_action);

    synchronized_action_performance_publisher.publish(synchronized_action_performance);
}