#pragma once

#include "ros/ros.h"

#include "ros_queue_experiments/AuvStates.h"

#include "localization_monitor.hpp"
#include "temperature_monitor.hpp"
#include "low_temperature_monitor.hpp"
#include "penalty_monitor.hpp"

class VirtualQueueMonitor
{
    public:
        VirtualQueueMonitor(ros::NodeHandle& nh);

    private:
        /**
         * @brief Node handle with namespace name resolution.
        */
        ros::NodeHandle nh_;

        /**
         * @brief Private node handle with name resolution from node's name.
        */
        ros::NodeHandle nhp_;

        /**
         * @brief Service client that fetches the estimation from the queue server.
        */
        ros::ServiceClient queue_server_stats_client_;
        /**
         * @brief ROS subscriber for the real state message that's used as an monitoring event.
        */
        ros::Subscriber real_state_sub_;

        /**
         * @brief Callback for the monitoring event (state message from the AUV system)
        */
        void realStateCallback(const ros_queue_experiments::AuvStates::ConstPtr& msg);

        /**
         * @brief Signal publisher to the optimal controller indicating that the monitoring sample is done.
         * When signal, this dependent controller is allowed to send its actions.
        */
        ros::Publisher monitoring_sample_done_pub_;

        /**
         * @brief Localization monitor that handles the computation and the publishing of the metrics of the localization.
        */
        LocalizationMonitor localization_monitor_;

        /**
         * @brief Temperature monitor that handles the computation and the publishing of the metrics of the temperature.
        */
        TemperatureMonitor temperature_monitor_;

        /**
         * @brief Temperature monitor for the lower bound of the temperature that handles the computation and the publishing of the metrics of the temperature.
        */
        LowTemperatureMonitor low_temperature_monitor_;

        /**
         * @brief Penalty monitor that handles the computation and the publishing of the metrics of the penalty.
        */
        PenaltyMonitor penalty_monitor_;
};