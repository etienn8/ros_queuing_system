#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "controller_queue_struct.hpp"

#include "ros_queue_msgs/PotentialAction.h"
#include "ros_queue_msgs/PotentialActionSet.h"

#include "rosparam_utils/xmlrpc_utils.hpp"
#include "rosparam_utils/parameter_package_fetch_struct.hpp"

using std::string;
using std::map;

class QueueController
{
    public:
        QueueController(ros::NodeHandle& nh);

    protected:
        /**
         * @brief Indicates if the controller received a valid configuration and was initialized.
        */
        bool is_initialized_ = false;

        ros::NodeHandle nh_;

        /**
         * @brief ROS publisher of the output action decided by the optimization.
        */
        ros::Publisher best_action_output_pub_;

        /**
         * @brief ROS service client used to get the current states of the queues in the queue server.
        */
        ros::ServiceClient server_state_client_;

        /**
         * @brief ROS topic publisher use to send manual changes to the virtual queues of a queue server.
         * It's mainly used when the inversed_control_and_update_steps_ is set to false since the queues must change
         * based on the metric computed for the best action.
        */
        ros::Publisher manual_virtual_queue_changes_;

        /**
         * @brief ROS Service client that whenever it's called, the virtual queues in the queue server
         * will update based on the current system's state.
        */
        ros::ServiceClient virtual_queues_trigger_;

        /**
         * @brief Map of all the queues used by the queue_controller from a queue_server. The key
         * is the queue's name and the value is all the usefull parameters of the queue to evaluate and 
         * udpate the queues.
        */
        map<string, std::unique_ptr<ControllerQueueStruct>> queue_list_;

        /**
         * @brief Populate the internal queue_list_ based on the parsed_queue_configs. It will also verify
         * if the queues exist in the queue server. If a queue doesn't exist, it won't be added to the queue_list.
         * @param parsed_queue_configs Parsed parameters from the queue_controller configuration from which
         * the queue_list_ will be populated.
         * @return Returns false if the queue server wasn't found and true otherwise.
        */
        bool populateQueueStructures(vector<xmlrpc_utils::ParameterPackageFetchStruct> parsed_queue_configs);

        // ===== Controller parameters =====
        /**
         * @brief If set to false, the controller will find an optimal next action to take and 
         * then update the virtual queues based on that optimal solution. If set to true, 
         * the controller will update the virtual queues based on the current of state of the 
         * system then find an optimal next action. Default: set to false
        */
        bool inversed_control_and_update_steps_ = false;
        
        /**
         * @brief Indicate the controller types. It defines the type of optimization and the used metrics.
        */
        string controller_type_;

        /**
         * @brief Indicate the period between each controller's call.
         * Used by the min_drift_plus_penalty controllers.
        */
        float controller_time_step_;

        /**
         * @brief Service server that a external node can call to trigger the renewal controller to execute.
         * Only used by the renewal_min_drift_plus_penalty controller.
        */
        ros::ServiceServer renewal_trigger_server_;

        /**
         * @brief Time after which, if the renewal_trigger_server_ was not call, the controller will be called.
         * Only used by the renewal_min_drift_plus_penalty controller.
        */
        float max_renewal_time = 0.0f;

        /**
         * @brief ROS service client that returns a set of possible actions that the controller will used
         * to compute its objetives metrics. The best solution will be a solution in this set.
        */
        ros::ServiceClient solution_space_client_;
        
        /**
         * @brief Service client that evaluates the optimization variable (penalty) given the state of
         * the system and an evaluating solution;
        */
        ros::ServiceClient penalty_service_client_;

        /**
         * @brief  The V parameter that is multplied with the penalty (like a weight). 
         * When a high penalty metric also increases the queue size, this parameter balances 
         * the tradeoff between a lower penalty or lower queue delays. A high V will 
         * prioritize the minimization  of the penalty while a low V will prioritize low queue 
         * sizes and delays. 
        */
        float v_parameter_;
        
        /**
         * @brief Name of the queue server node linked to the queue_controller. Mainly used as a suffix
         * to solve all the ROS topic and ROS service names provided by the queue server. 
        */
        string queue_server_name_;

        /**
         *  @brief List of all the current potential actions that will be evaluated.
        */
        std::vector<ros_queue_msgs::PotentialAction> current_action_set_;

        /**
         * @brief List of all the queue's expected float parameters. Only used at config time.
        */
        std::vector<string> queue_float_parameter_names_{"weight"};

        /**
         * @brief List of all the queue's expected string parameters. Only used at config time.
        */
        std::vector<string> queue_string_parameter_names_{"expected_arrival_service_name", "expected_departure_service_name"};
};
