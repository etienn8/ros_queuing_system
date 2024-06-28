#pragma once

#include "auv_forward_declarations.hpp"

#include "ros/ros.h"

#include <memory>

#include "ros_queue_experiments/metrics/low_temperature_services.hpp"
#include "ros_queue_experiments/metrics/metric_services.hpp"
#include "ros_queue_experiments/metrics/temperature_services.hpp"
#include "ros_queue_experiments/metrics/renewal_time_services.hpp"
#include "ros_queue_experiments/metrics/localization_services.hpp"
#include "ros_queue_experiments/metrics/task_publisher.hpp"
#include "ros_queue_experiments/metrics/penalty_services.hpp"

#include "ros_queue_experiments/AuvStates.h"
#include "ros_queue_experiments/GetRealAUVStates.h"
#include "ros_queue_experiments/SendNewAUVCommand.h"

#include "ros_queue_msgs/PotentialTransmissionVectorSpaceFetch.h"

#include "std_msgs/Empty.h"

class AUVSystem
{
    public:
        AUVSystem(ros::NodeHandle& nh);

        // Metric service servers
        std::shared_ptr<PenaltyServices> penalty_metric_services_;
        std::shared_ptr<RenewalTimeServices> expected_time_services_; 
        std::shared_ptr<TemperatureServices> temperature_services_;
        std::shared_ptr<LowTemperatureServices> low_temperature_services_;
        std::shared_ptr<LocalizationServices> localization_services_;
        std::shared_ptr<TaskPublisher> task_services_;

    private:
        /**
         * @brief Current states of the auv and its metrics.
        */
        std::shared_ptr<AUVStateManager> auv_state_manager_;

        /**
         * @brief ROS private node handle to manage the servers.
        */
        ros::NodeHandle nh_;


        /**
         * @brief ROS node handle with namesapce name resolution.
        */
        ros::NodeHandle ns_nh_;

        /**
         * @brief ROS publisher for the AUV state.
        */
        ros::Publisher state_pub_;

        /**
         * @brief Service to get all the possible actions
        */
        ros::ServiceServer action_set_service_;

        /**
         * @brief Callback the service to return all the possible actions
        */
        bool potentialActionSetCallback(ros_queue_msgs::PotentialTransmissionVectorSpaceFetch::Request& req,
                                        ros_queue_msgs::PotentialTransmissionVectorSpaceFetch::Response& res);
        
        /**
         * @brief Service to get the current internal states of the UAV.
        */
        ros::ServiceServer auv_state_service_;

        /**
         * @brief Callback the service to return the current internal states of the UAV.
         * @param req Request to get the current states of the UAV. Is empty.
         *  @param res Response to get the current states of the UAV.
        */
        bool auvStateCallback(ros_queue_experiments::GetRealAUVStates::Request& req,
                              ros_queue_experiments::GetRealAUVStates::Response& res);

        /**
         * @brief Subscriber that listens to the starting step of the controller to publish the auv states for the 
         * monitoring system.
        */
        ros::Subscriber control_started_sub_;

        /**
         * @brief Callback called when the controller has started (right after sending its updates to the queue server
         * of step inverted controller) and that publishes auv state msg for the monitoring. This moment is the best for 
         * sampling the auv state for monitoring.
         * @param msg Empyt message which is the signal used by queue controller to signal the beginning of its loop.
         * @return True if the callback was successful. Which is always true in this case.
        */
         void stateMonitoringCallback(const std_msgs::Empty::ConstPtr& msg);

        /**
         * @brief Service to set the new command to the system.
        */
        ros::ServiceServer command_service_;

        /**
         * @brief Callback the service to set the new command to the system and publish the auv states.
         * @param req Request to set the new command to the system.
         * @param res Return the time to complete the command.
        */
        bool commandCallback(ros_queue_experiments::SendNewAUVCommand::Request& req,
                             ros_queue_experiments::SendNewAUVCommand::Response& res);
};