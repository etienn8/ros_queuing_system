#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros/ros.h"

#include "controller_queue_struct.hpp"

#include "ros_queue_msgs/FloatRequest.h"
#include "ros_queue_msgs/QueueInfoFetch.h"
#include "ros_queue_msgs/QueueServerStateFetch.h"
#include "ros_queue_msgs/VirtualQueueChangesList.h"

#include "std_srvs/Empty.h"

#include "rosparam_utils/xmlrpc_utils.hpp"
#include "rosparam_utils/parameter_package_fetch_struct.hpp"

#include "queue_server/queue_server_utils.hpp"

using std::string;
using std::map;

template <typename TActionSetType>
struct ActionTrait {
  using ActionType = typename TActionSetType::_action_set_type::value_type;
};

template<typename TMetricControlPredictionSrv, typename TPotentialActionSetMsg, typename TPotentialActionSetSrv>
class QueueController
{
    public:
        typedef typename ActionTrait<TPotentialActionSetMsg>::ActionType ActionType;

        enum ControllerType
        {
            DriftPlusPenalty,
            RenewalDriftPlusPenalty
        };

        QueueController(ros::NodeHandle& nh): nh_(nh)
        {
            bool can_create_controller = true;

            string controller_type_name;
            if(nh.getParam("controller_type", controller_type_name))
            {
                if(controller_type_name == "min_drift_plus_penalty")
                {
                    controller_type_ = ControllerType::DriftPlusPenalty;
                }
                else if (controller_type_name == "renewal_min_drift_plus_penalty")
                {
                    controller_type_ = ControllerType::RenewalDriftPlusPenalty;
                }
                else
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Unknown controller type:" << controller_type_name <<". Supported types: min_drift_plus_penalty and renewal_min_drift_plus_penalty"); 
                }
            }
            else
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the controller type");
            }

            if(controller_type_ == ControllerType::DriftPlusPenalty)
            {
                if(!nh.getParam("time_step", controller_time_step_))
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Missing the controller time step");
                }
            }

            if(!nh.getParam("inverse_control_and_steps", inversed_control_and_update_steps_))
            {
                ROS_WARN_STREAM("Missing the flag inverse_control_and_steps that indicates if the control and update steps are inverted. Will be set to false.");
            }

            string solution_space_service_name;
            if(nh.getParam("solution_space_service_name", solution_space_service_name))
            {
                solution_space_client_ = nh_.serviceClient<TPotentialActionSetSrv>(solution_space_service_name, true);
            }
            else
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the solution_space_service_name.");
            }

            string penalty_service_name;
            if(nh.getParam("penalty_service_name", penalty_service_name))
            {
                penalty_service_client_ = nh_.serviceClient<TMetricControlPredictionSrv>(penalty_service_name, true);
            }
            else
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the penalty_service_name.");
            }

            if(nh.getParam("v_parameter", v_parameter_))
            {
                if(v_parameter_< 0.0f)
                {
                    ROS_ERROR_STREAM("v_parameter has negative value: "<< v_parameter_ <<". Queue controllers expects non-negative V parameters.");
                }
            }
            else
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the v_parameter.");
            }

            if(!nh.getParam("queue_server_name", queue_server_name_))
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the queue_server_name.");
            }

            // Parse queue parameters
            xmlrpc_utils::ParameterPackageFetchStruct::OptionsStruct options;

            options.string_parameter_name_list = std::vector<string>{"expected_arrival_service_name", "expected_departure_service_name"};
            options.float_parameter_name_list = std::vector<string>{"weight"};
            options.bool_parameter_name_list = std::vector<string>{"arrival_action_dependent", "departure_action_dependent"};

            const xmlrpc_utils::ParameterPackageFetchStruct fetch_struct(options);
            
            vector<xmlrpc_utils::ParameterPackageFetchStruct> parsed_queue_configs= 
                            xmlrpc_utils::fetchMatchingParametersFromList(nh_, ros::this_node::getName(),
                                                                        "queue_list", fetch_struct);

            // Does the queue server exist and transfer queue_configs in internal queue structs
            if(!populateQueueStructures(parsed_queue_configs))
            {
                can_create_controller = false;
            }

            // Intialize controller
            if (can_create_controller)
            {   
                // Connect to queue_server for queue sizes
                server_state_client_ = nh_.serviceClient<ros_queue_msgs::QueueServerStateFetch>("/" + queue_server_name_ + "/get_server_state", true);

                // Connect to queue_server for queue server udpates (depends on steps order)
                if (!inversed_control_and_update_steps_)
                {   
                    manual_virtual_queue_changes_ = nh_.advertise<ros_queue_msgs::VirtualQueueChangesList>("/" + queue_server_name_ + "/virtual_queue_manual_changes", 1000);
                }
                else
                {
                    virtual_queues_trigger_ = nh_.serviceClient<std_srvs::Empty>("/" + queue_server_name_ + "/trigger_service", true);
                }

                // Create ouptut topic
                best_action_output_pub_ = nh_.advertise<ActionType>("best_action", 1000);
                
                // Initialize the triggers
                if (controller_type_ == ControllerType::DriftPlusPenalty)
                {
                    periodic_trigger_timer_ = nh_.createTimer(ros::Duration(controller_time_step_), 
                                                            &QueueController::minDriftPlusPenaltyCallback, this);
                }

                is_initialized_ = true;
            }
        }

        struct ObjectiveParameter
        {
            float expected_arrivals;
            float expected_departures;

            float current_size;
        };

        struct ActionParameters
        {
            ActionType action;
            
            float penalty;
            
            std::map<string, ObjectiveParameter> queue_parameters;

            float expected_renewal_time;

            float cost = 0.0f;
        };

    private:
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
        bool populateQueueStructures(vector<xmlrpc_utils::ParameterPackageFetchStruct>& parsed_queue_configs)
        {
            // Create structure to parse the queue params from the queue server
            ROS_INFO_STREAM("Waiting for queue serve named " << queue_server_name_ << " to be online. Waiting at most 10 seconds.");
            
            if (ros::service::waitForService("/"+queue_server_name_+"/get_server_state", ros::Duration(10)))
            {
                ROS_INFO_STREAM("Queue server found. Resuming queue controller initialization.");

                for(auto parsed_queue_config_it = parsed_queue_configs.begin(); parsed_queue_config_it != parsed_queue_configs.end(); ++parsed_queue_config_it)
                {
                    const string& queue_name_temp = parsed_queue_config_it->param_package_name_;
                    
                    ros_queue_msgs::QueueInfoFetch queue_info_srv;

                    if(ros::service::call(queue_server_name_+"/"+queue_name_temp+"/getQueueInfo", queue_info_srv))
                    {
                        auto new_controller_struct = std::make_unique<ControllerQueueStruct>();

                        new_controller_struct->queue_name_ = queue_name_temp;
                        new_controller_struct->is_virtual_ = queue_info_srv.response.info.is_virtual;
                        
                        bool is_queue_parameters_ok = true;

                        // Verify if the weight parameter of a queue is set 
                        auto weight_pair_it = parsed_queue_config_it->float_params_.find("weight");
                        if (weight_pair_it != parsed_queue_config_it->float_params_.end() && weight_pair_it->second.second)
                        {
                            new_controller_struct->weight_ = weight_pair_it->second.first;
                        }
                        else
                        {
                            is_queue_parameters_ok = false;
                            ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing his weight parameter");
                        }

                        // Verify if the expected_arrival_service_name is set
                        auto service_name_pair_it = parsed_queue_config_it->string_params_.find("expected_arrival_service_name");
                        if (service_name_pair_it != parsed_queue_config_it->string_params_.end() && service_name_pair_it->second.second)
                        {
                            string& service_name_temp = service_name_pair_it->second.first;
                            
                            // Verify if it should subscribe to a srv with an action request.
                            auto action_dependent_flag_it = parsed_queue_config_it->bool_params_.find("arrival_action_dependent");
                            if (action_dependent_flag_it != parsed_queue_config_it->bool_params_.end())
                            {
                                if(action_dependent_flag_it->second.first)
                                {
                                    new_controller_struct->expected_arrival_service_ = 
                                                            nh_.serviceClient<TMetricControlPredictionSrv>(service_name_temp, true);
                                }
                                else
                                {
                                    // The parameter was not found in the rosparam file
                                    if (!action_dependent_flag_it->second.second)
                                    {
                                        ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing its arrival_action_dependent parameter. Will be set to false.");
                                    }
                                    new_controller_struct->arrival_independent_from_action_service_ = 
                                                            nh_.serviceClient<ros_queue_msgs::FloatRequest>(service_name_temp, true);
                                }
                            }

                        }
                        else
                        {
                            is_queue_parameters_ok = false;
                            ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing his expected_arrival_service_name parameter");
                        }

                        // Verify if the expected_departure_service_name is set
                        service_name_pair_it = parsed_queue_config_it->string_params_.find("expected_departure_service_name");
                        if (service_name_pair_it != parsed_queue_config_it->string_params_.end() && service_name_pair_it->second.second)
                        {
                            string& service_name_temp = service_name_pair_it->second.first;
                            
                            // Verify if it should subscribe to a srv with an action request.
                            auto action_dependent_flag_it = parsed_queue_config_it->bool_params_.find("departure_action_dependent");
                            if (action_dependent_flag_it != parsed_queue_config_it->bool_params_.end())
                            {
                                if(action_dependent_flag_it->second.first)
                                {
                                    new_controller_struct->expected_departure_service_ = 
                                                            nh_.serviceClient<TMetricControlPredictionSrv>(service_name_temp, true);
                                }
                                else
                                {
                                    // The parameter was not found in the rosparam file
                                    if (!action_dependent_flag_it->second.second)
                                    {
                                        ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing its departure_action_dependent parameter. Will be set to false.");
                                    }
                                    new_controller_struct->departure_independent_from_action_service_ = 
                                                            nh_.serviceClient<ros_queue_msgs::FloatRequest>(service_name_temp, true);
                                }
                            }

                        }
                        else
                        {
                            is_queue_parameters_ok = false;
                            ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing his expected_arrival_service_name parameter");
                        }

                        // Add queue to internal structure if all necessary parameters are defined
                        if(is_queue_parameters_ok)
                        {
                            queue_list_[queue_name_temp] = std::move(new_controller_struct);
                        }
                    }
                    else
                    {
                        ROS_WARN_STREAM("Expected queue " << queue_name_temp << " from the queue controller wasn't found in the queue server " << queue_server_name_ << ". The queue won't be considered by the controller.");
                    }
                }

            }
            else
            {
                ROS_ERROR_STREAM("Queue server not found. Stopping queue controller initialization.");
                return false;
            }

            return true;
        }


        // ===== Trigger attributes =====
        /**
         * @brief Timer that periodically calls the queue controller sequence.
        */
        ros::Timer periodic_trigger_timer_;

        /**
         * @brief Callback of the min-drift-plus-penalty algorithm that is called by a periodic timer.
         * @param time_event Information of the time event.
        */
        void minDriftPlusPenaltyCallback(const ros::TimerEvent& time_event)
        {
            ROS_DEBUG_STREAM("Queue controller: Staring control loop of "<< ros::this_node::getName());
            
            if (inversed_control_and_update_steps_)
            {
                updateVirtualQueuesBasedOnCurrentState();
            }

            TPotentialActionSetMsg action_set =  getActionSet();
            std::vector<ActionParameters> action_parameters_list(action_set.action_set.size());
            if(getParametersForControlStep(action_set, action_parameters_list))
            {
                ActionParameters best_action_parameters = computeMinDriftPlusPenalty(action_parameters_list);
                
                sendBestCommand(best_action_parameters.action);

                if (!inversed_control_and_update_steps_)
                {
                    updateVirtualQueuesBasedOnBestAction(best_action_parameters);
                }
            }
        }


        // ===== Controller steps ======
        /**
         * @brief Calls the service to get the all the potential actions that will be evaluated.
         * @return The set of potential actions.
        */
        TPotentialActionSetMsg getActionSet()
        {
            ROS_DEBUG("Calling action set service");

            TPotentialActionSetSrv potential_set_srv;
            
            if(!solution_space_client_.call(potential_set_srv))
            {
                ROS_WARN_STREAM_THROTTLE(2, "Failed to call the service that retrieves the action set");
            }

            return potential_set_srv.response.action_set;
        }
        
        /**
         * @brief Evaluates the penalty, the queues sizes and the queue changes for each action.
         * @param action_set The set of potential actions from which all metrics will be predicted.
         * @param action_parameters_output List of all the penalty and queues parameters for each action. Used as an output.
         * @return Returns true if the parameters are valid.
        */
         bool getParametersForControlStep(TPotentialActionSetMsg& action_set_msg, std::vector<ActionParameters>& action_parameters_output)
        {
            const int& size_of_actions = action_set_msg.action_set.size();

            // Populate actions
            for (int action_index = 0; action_index < size_of_actions; ++action_index)
            {
                action_parameters_output[action_index].action = action_set_msg.action_set[action_index];
            }

            bool are_parameters_valid = true;

            // Penalty service
            TMetricControlPredictionSrv penalty_prediction_srv;
            penalty_prediction_srv.request.action_set = action_set_msg;
            if (penalty_service_client_.call(penalty_prediction_srv))
            {
                if (penalty_prediction_srv.response.predictions.size() != size_of_actions)
                {
                    are_parameters_valid = false;
                    ROS_WARN_STREAM_THROTTLE(2, "Returned penalty array doesn't contain the same amount of elements has the action set (expected " << size_of_actions << ", received "<< penalty_prediction_srv.response.predictions.size() <<")");
                }
            }
            else
            {
                are_parameters_valid = false;
                ROS_WARN_STREAM_THROTTLE(2, "Failed to call the penalty evalution service");
            }

            // Get queue sizes
            ros_queue_msgs::QueueServerStateFetch server_state_msg;
            if(!server_state_client_.call(server_state_msg))
            {
                are_parameters_valid = false;
                ROS_WARN_STREAM_THROTTLE(2, "Failed to call the queue server state fetch# service");
            }

            // Expected time
            // TODO

            // Queues services
            for (auto queue_it = queue_list_.begin(); queue_it != queue_list_.end(); ++queue_it)
            {
                const string& queue_name = queue_it->first;
                
                // Arrivals
                if(queue_it->second->expected_arrival_service_.isValid())
                {
                    TMetricControlPredictionSrv arrival_predictions;
                    arrival_predictions.request.action_set = action_set_msg;

                    if(queue_it->second->expected_arrival_service_.call(arrival_predictions))
                    {
                        const int& returned_size = arrival_predictions.response.predictions.size();
                        
                        if(returned_size == size_of_actions)
                        {
                            for (int action_index =0; action_index < size_of_actions; ++action_index)
                            {
                                action_parameters_output[action_index].queue_parameters[queue_name].expected_arrivals = arrival_predictions.response.predictions[action_index];
                            }
                        }
                        else
                        {
                            are_parameters_valid = false;
                            ROS_WARN_STREAM_THROTTLE(2, "Returned prediction array doesn't contain the same amount of elements has the action set (expected " << size_of_actions << ", received "<< returned_size <<")");
                        }
                    }
                    else
                    {
                        are_parameters_valid = false;
                        ROS_WARN_STREAM_THROTTLE(2, "Failed to call the expected arrival service named: " << queue_it->second->expected_arrival_service_.getService());
                    }
                }
                else if (queue_it->second->arrival_independent_from_action_service_.isValid())
                {
                    ros_queue_msgs::FloatRequest arrival_prediction;
                    if(queue_it->second->arrival_independent_from_action_service_.call(arrival_prediction))
                    {
                        for (int action_index =0; action_index < size_of_actions; ++action_index)
                        {
                            action_parameters_output[action_index].queue_parameters[queue_name].expected_arrivals = arrival_prediction.response.value;
                        }
                    }
                    else
                    {
                        are_parameters_valid = false;
                        ROS_WARN_STREAM_THROTTLE(2, "Failed to call the expected arrival service independent from actions named: " << queue_it->second->arrival_independent_from_action_service_.getService());
                    }
                }
                else
                {

                    are_parameters_valid = false;
                    ROS_WARN_STREAM_THROTTLE(2, "The arrival evaluation service of the queue "<< queue_it->first <<"is not valid");
                }

                // Departures
                if(queue_it->second->expected_departure_service_.isValid())
                    {
                        TMetricControlPredictionSrv departure_predictions;
                        departure_predictions.request.action_set = action_set_msg;

                        if(queue_it->second->expected_departure_service_.call(departure_predictions))
                        {
                            const int& returned_size = departure_predictions.response.predictions.size();
                            
                            if(returned_size == size_of_actions)
                            {
                                for (int action_index =0; action_index < size_of_actions; ++action_index)
                                {
                                    action_parameters_output[action_index].queue_parameters[queue_name].expected_departures = departure_predictions.response.predictions[action_index];
                                }
                            }
                            else
                            {
                                are_parameters_valid = false;
                                ROS_WARN_STREAM_THROTTLE(2, "Returned prediction array doesn't contain the same amount of elements has the action set (expected " << size_of_actions << ", received "<< returned_size <<")");
                            }
                        }
                        else
                        {
                            are_parameters_valid = false;
                            ROS_WARN_STREAM_THROTTLE(2, "Failed to call the expected departure service named: " << queue_it->second->expected_departure_service_.getService());
                        }
                    }
                    else if (queue_it->second->departure_independent_from_action_service_.isValid())
                    {
                        ros_queue_msgs::FloatRequest departure_prediction;

                        if(queue_it->second->departure_independent_from_action_service_.call(departure_prediction))
                        {
                            for (int action_index =0; action_index < size_of_actions; ++action_index)
                            {
                                action_parameters_output[action_index].queue_parameters[queue_name].expected_departures = departure_prediction.response.value;
                            }
                        }
                        else
                        {
                            are_parameters_valid = false;
                            ROS_WARN_STREAM_THROTTLE(2, "Failed to call the expected depature service independent from actions named: " << queue_it->second->departure_independent_from_action_service_.getService());
                        }
                    }
                    else
                    {
                        are_parameters_valid = false;
                        ROS_WARN_STREAM_THROTTLE(2, "The departure evaluation service of the queue "<< queue_it->first <<"is not valid");
                    }

            }
            // Populate the output if the parameters are valid
            if (are_parameters_valid)
            {
                for (int action_index = 0; action_index < size_of_actions; ++action_index)
                {
                    action_parameters_output[action_index].penalty = penalty_prediction_srv.response.predictions[action_index];
                    for (auto queue_it = queue_list_.begin(); queue_it != queue_list_.end(); ++queue_it)
                    {
                        const string& queue_name = queue_it->first;
                        ObjectiveParameter& queue_parameters_out = action_parameters_output[action_index].queue_parameters[queue_name];

                        for (auto queue_it = server_state_msg.response.queue_server_state.queue_sizes.begin(); 
                                 queue_it != server_state_msg.response.queue_server_state.queue_sizes.end(); ++queue_it)
                        {
                            if(queue_name == queue_it->queue_name)
                            {
                                queue_parameters_out.current_size = queue_it->current_size;
                                break;
                            }
                        }
                    }
                }
            }

             return are_parameters_valid;
        }
        
        /**
         * @brief Used the min drift-plus-penalty algorithm to compute the action that minimize the penalty and stabilizes all the queues (if possible).
         * @param action_parameters_list All the parameters and variables for the penalty and the queues for each actions.
         * @return Returns the best action including all the metrics.
        */
        ActionParameters computeMinDriftPlusPenalty(std::vector<ActionParameters>& action_parameters_list)
        {
            auto best_action = action_parameters_list.begin();

            bool is_first_cost = true;

            for (auto action_parameters_it = action_parameters_list.begin(); action_parameters_it != action_parameters_list.end(); ++action_parameters_it)
            {
                action_parameters_it->cost = v_parameter_*action_parameters_it->penalty;
                
                for(auto queue_it = action_parameters_it->queue_parameters.begin(); queue_it != action_parameters_it->queue_parameters.end(); ++queue_it)
                {
                    const string& queue_name = queue_it->first;
                    action_parameters_it->cost += queue_list_[queue_name]->weight_ * 
                                                  queue_it->second.current_size * 
                                                  (queue_it->second.expected_arrivals - queue_it->second.expected_departures);
                }

                if (is_first_cost)
                {
                    is_first_cost = false;
                    best_action = action_parameters_it;
                }
                else if(action_parameters_it->cost < best_action->cost)
                {
                    best_action = action_parameters_it;
                }
            }

            return *best_action;
        }
        
        /**
         * @brief Publish the best action.
         * @param best_action The best action that was computed
        */
        void sendBestCommand(ActionType& best_action)
        {
            best_action_output_pub_.publish(best_action);
        }
        
        /**
         * @brief Send a signal to the queue server to update the virtual queues based on the 
         * current state of the system.
        */
        void updateVirtualQueuesBasedOnCurrentState()
        {

        }

        /**
         * @brief Send a message to the queue server to update the virual queues based on the
         * metrics of the best action
        */
        void updateVirtualQueuesBasedOnBestAction(ActionParameters& best_action_parameters)
        {
            ros_queue_msgs::VirtualQueueChangesList change_list;

            for (auto queue_it = queue_list_.begin(); queue_it != queue_list_.end(); ++queue_it)
            {
                if(queue_it->second->is_virtual_)
                {
                    const string& queue_name = queue_it->first;
                    ros_queue_msgs::VirtualQueueChanges queue_changes;
                    
                    queue_changes.queue_name = queue_name;
                    queue_changes.arrival = best_action_parameters.queue_parameters[queue_name].expected_arrivals;
                    queue_changes.departure = best_action_parameters.queue_parameters[queue_name].expected_departures;
                    
                    change_list.list.push_back(std::move(queue_changes));
                }
            }

            manual_virtual_queue_changes_.publish(change_list);
        }


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
        ControllerType controller_type_;

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
};
