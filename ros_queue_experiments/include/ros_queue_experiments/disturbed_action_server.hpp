#pragma once

#include <memory>

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "ros_queue_msgs/TransmissionVectorAction.h"

class DisturbedActionServer
{
    public:
        DisturbedActionServer(ros::NodeHandle& nh);

    private:
        enum class PerturbationType
        {
            NotMoving =0,
            OffsetPlusOne
        };

        /**
         * @brief ROS node handle to manage the servers.
         */
        ros::NodeHandle nh_;

        /**
         * @brief Action server to receive the action from the queue controller.
        */
        actionlib::SimpleActionServer<ros_queue_msgs::TransmissionVectorAction> action_server_;

        /**
         * @brief Callaback when a action is received. It adds a perturbation if needed and send it to the AUV system.
        */ 
        void executeActionCallback(const ros_queue_msgs::TransmissionVectorGoalConstPtr& goal);

        /**
         * @brief Control steps between each pertubations.
        */
        int perturbation_at_each_x_control_steps_ = 0;

        /**
         * @brief Number of steps since the last perturbation.
        
        */
        int steps_since_last_perturbation_ = 0;

        /**
         * @brief Type of perturbation.
        */
        PerturbationType perturbation_type_ = PerturbationType::NotMoving;
        
        /**
         * @brief Service client to get the current AUV states
        */
        ros::ServiceClient auv_state_client_;

        /**
         * @brief Service client to send the disturbed action to the AUV system.
        */
        ros::ServiceClient auv_action_client_;
};