#pragma once

#include "auv_forward_declarations.hpp"

#include "ros/ros.h"

#include <memory>

#include "ros_queue_experiments/metrics/metric_services.hpp"
#include "ros_queue_experiments/metrics/temperature_services.hpp"
#include "ros_queue_experiments/metrics/renewal_time_services.hpp"
#include "ros_queue_experiments/metrics/localization_services.hpp"
#include "ros_queue_experiments/metrics/task_publisher.hpp"
#include "ros_queue_experiments/metrics/penalty_services.hpp"

#include "ros_queue_experiments/AuvStates.h"

class AUVSystem
{
    public:
        AUVSystem(ros::NodeHandle& nh);

        // Metric service servers
        std::shared_ptr<PenaltyServices> penalty_metric_services_;
        std::shared_ptr<RenewalTimeServices> expected_time_services_; 
        std::shared_ptr<TemperatureServices> temperature_services_;
        std::shared_ptr<LocalizationServices> localization_services_;
        std::shared_ptr<TaskPublisher> task_services_;

    private:
        /**
         * @brief Current states of the auv and its metrics.
        */
        std::shared_ptr<AUVStateManager> auv_state_manager_;

        /**
         * @brief ROS node handle to manage the servers.
        */
        ros::NodeHandle nh_;

        /**
         * @brief ROS publisher for the AUV state.
        */
        ros::Publisher state_pub_;

        /**
         * @brief AUV system spin to propagate the dynamic of the states and publish metrics. 
        */
        void spin();

        /**
         * @brief Update the states of the metrics based on the current of the UAV.
        */
        void stateTransitions();
};