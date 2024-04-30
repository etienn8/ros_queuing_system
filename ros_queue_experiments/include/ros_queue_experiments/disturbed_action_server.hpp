#pragma once

#include <memory>
#include <mutex>

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
         * @brief Callback when a command is received from the queue controller. 
         * It sets the states of the AUV system and waits for the renewal. If a new goal is received
         * while the old one is still active, the old one is aborted.
        */
        void commandReceivedCallback();
        
        /**
         * @brief Callback when a preempt is received from the queue controller.
        */
        void preemptActionCallback();
        
        /**
         * @brief Control steps between each pertubations.
        */
        int perturbation_at_each_x_control_steps_ = 0;

        /**
         * @brief Timer that sends a sucess for the current action based on the renewal the last renewal received.
         * It's reset a each new received action. 
        */
        ros::Timer send_sucess_timer_;

        /**
         * @brief Callback called when the timer expires. It sends a success to the queue controller for the current action.
         * @param event Timer event.
        */
        void sendSuccessCallback(const ros::TimerEvent& event);

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

        /**
         * @brief ROS publisher to indicated the desired and the real taken actions.
        */
        ros::Publisher action_performance_publisher_;

        /**
         * @brief Flag that indicates if the action server is dummy. 
         * If it's dummy, no services will be defined to send over a auv system
        */
        bool is_dummy_ = false;
};