#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros/ros.h"

#include "controller_queue_struct.hpp"
#include "queue_controller_utils.hpp"

#include "ros_queue_msgs/FloatRequest.h"
#include "ros_queue_msgs/QueueInfoFetch.h"
#include "ros_queue_msgs/QueueServerStateFetch.h"
#include "ros_queue_msgs/VirtualQueueChangesList.h"

#include <actionlib/client/simple_action_client.h>

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

template<typename TMetricControlPredictionSrv, typename TPotentialActionSetMsg, typename TPotentialActionSetSrv, typename TActionLibOutputType, typename TActionLibOutputGoalType>
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

            if(controller_type_ == ControllerType::RenewalDriftPlusPenalty)
            {
                if(!nh.getParam("max_renewal_time", max_renewal_time_))
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Missing the max_renewal_time");
                }

                if(!nh.getParam("min_renewal_time", min_renewal_time_))
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Missing the min_renewal_time");
                }

                if(!nh.getParam("is_penalty_renewal_dependent", is_penalty_renewal_dependent_))
                {
                    ROS_WARN_STREAM("Missing the flag is_penalty_renewal_dependent that indicates if the penalty is dependent on the renewal time. Will be set to false.");
                }

                string expected_renewal_time_service_name;
                if(!nh.getParam("expected_renewal_time_service_name", expected_renewal_time_service_name) || expected_renewal_time_service_name.empty())
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Missing the expected_renewal_time_service_name.");
                }
                else
                {
                    renewal_service_client_ = nh_.serviceClient<TMetricControlPredictionSrv>(expected_renewal_time_service_name, true);
                    
                    ROS_INFO_STREAM("Waiting for the expected renewal time service named :" << expected_renewal_time_service_name);
                    renewal_service_client_.waitForExistence();
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
                
                ROS_INFO_STREAM("Waiting for the solution space client named :" << solution_space_service_name);
                solution_space_client_.waitForExistence();
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
                
                ROS_INFO_STREAM("Waiting for the penalty space client named :" << penalty_service_name);
                penalty_service_client_.waitForExistence();
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
            options.bool_parameter_name_list = std::vector<string>{"arrival_action_dependent", "departure_action_dependent", "arrival_renewal_dependent", "departure_renewal_dependent"};

            const xmlrpc_utils::ParameterPackageFetchStruct fetch_struct(options);
            
            vector<xmlrpc_utils::ParameterPackageFetchStruct> parsed_queue_configs= 
                            xmlrpc_utils::fetchMatchingParametersFromList(nh_, ros::this_node::getName(),
                                                                        "queue_list", fetch_struct);

            // Does the queue server exist and transfer queue_configs in internal queue structs
            if(!populateQueueStructures(parsed_queue_configs))
            {
                can_create_controller = false;
            }

            string action_server_name;
            if(!nh.getParam("action_server_name", action_server_name))
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the action_server_name.");
            }

            // Intialize controller
            if (can_create_controller)
            {   
                // Connect to queue_server for queue sizes
                server_state_client_ = nh_.serviceClient<ros_queue_msgs::QueueServerStateFetch>("/" + queue_server_name_ + "/get_server_state", true);

                ROS_INFO_STREAM("Waiting for the server service named :" << server_state_client_.getService());
                server_state_client_.waitForExistence();

                // Connect to queue_server for queue server udpates (depends on steps order)
                if (!inversed_control_and_update_steps_)
                {   
                    manual_virtual_queue_changes_ = nh_.advertise<ros_queue_msgs::VirtualQueueChangesList>("/" + queue_server_name_ + "/virtual_queue_manual_changes", 1000);
                }
                else
                {
                    virtual_queues_trigger_ = nh_.serviceClient<std_srvs::Empty>("/" + queue_server_name_ + "/trigger_service", true);
                    ROS_INFO_STREAM("Waiting for the server service named :" << virtual_queues_trigger_.getService());
                    virtual_queues_trigger_.waitForExistence();
                }

                // Create ouptut topic
                best_action_client_ =  std::make_unique<actionlib::SimpleActionClient<TActionLibOutputType>>(action_server_name, true);   

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

        /**
         * @brief Main loop of the queue controller. It will call the control sequence based on the controller type.
        */
        void spin()
        {
            if (!is_initialized_)
            {
                ROS_ERROR_STREAM_THROTTLE(2, "Queue controller wasn't initialized. Stopping queue controller.");
                return;
            }

            if (controller_type_ == ControllerType::DriftPlusPenalty)
            {
                static ros::Rate loop_rate(1.0f/controller_time_step_);
                while(ros::ok())
                {
                    controllerCallback();
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
            else if (controller_type_ == ControllerType::RenewalDriftPlusPenalty)
            {
                while(ros::ok())
                {
                    if(!best_action_client_waited_)
                    {
                        if(!best_action_client_->waitForServer(ros::Duration(5.0)))
                        {
                            ROS_WARN_STREAM("The action server named wasn't found. Queue controller will be running with a perdiod of max_renewal_time ("<< max_renewal_time_ << "s) between each control step and will continue to try to connect to server.");
                        }

                        best_action_client_waited_ = true;
                    }

                    static bool first_renewal = true;
                    if(first_renewal)
                    {
                        first_renewal = false;
                        last_renewal_time_point = ros::Time::now();
                    }

                    // Compute the real elapsed time since the last renewal.
                    last_renewal_time_ = (ros::Time::now() - last_renewal_time_point).toSec();
                    // Compute the controller
                    controllerCallback();

                    last_renewal_time_point = ros::Time::now();

                    if(best_action_client_->isServerConnected())
                    {
                        // Wait for the best action to be reached or wait for the max_renewal_time
                        bool finished_before_max_time =  best_action_client_->waitForResult(ros::Duration(max_renewal_time_));
                        
                        // We abandon the last goal if its still on going.
                        if (!finished_before_max_time)
                        {
                            best_action_client_->cancelGoal();
                        }

                        double elapsed_time = (ros::Time::now() - last_renewal_time_point).toSec();

                        // Wait for t_min is reached if the goal was reached before.
                        if (elapsed_time < min_renewal_time_)
                        {
                            ros::Duration(min_renewal_time_ - elapsed_time).sleep();
                        }
                    }
                    else
                    {
                        ros::Duration(max_renewal_time_).sleep();      
                    }

                    ROS_DEBUG_STREAM("Time since last renewal: " << last_renewal_time_);
                }
            }
        }

    private:
        /**
         * @brief Indicates if the controller received a valid configuration and was initialized.
        */
        bool is_initialized_ = false;

        ros::NodeHandle nh_;

        /**
         * @brief Time at which the last renewal was done. Used by the renewal_min_drift_plus_penalty controller.
        */
        ros::Time last_renewal_time_point;

        /**
         * @brief Last elapsed time between the last renewal and the current time in seconds. Used by the renewal_min_drift_plus_penalty controller
         * and only used with virtual updates that are based on the current state of the system (inversed_control_and_update_steps_ is set to true).
        */
        double last_renewal_time_;

        /**
         * @brief ROS service client used to computed the expected time ot execute each given action.
        */
       ros::ServiceClient renewal_service_client_;

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
         * @brief Action lib client that sends the optimal action to a server.
        */
         std::unique_ptr<actionlib::SimpleActionClient<TActionLibOutputType>> best_action_client_;

        /**
         * @brief Flag that indicates if the best action client already waited for the server to be online. 
        */
        bool best_action_client_waited_ = false; 

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
                            
                            // Verify if the arrivals of the queue is affected by the renewal time.
                            auto arrival_renewal_flag_it = parsed_queue_config_it->bool_params_.find("arrival_renewal_dependent");
                            if (controller_type_ == ControllerType::RenewalDriftPlusPenalty && 
                                arrival_renewal_flag_it != parsed_queue_config_it->bool_params_.end() && 
                                arrival_renewal_flag_it->second.second)
                            {
                                new_controller_struct->is_arrival_renewal_dependent = arrival_renewal_flag_it->second.first;
                            }

                            // Verify if it should subscribe to a srv with an action request.
                            auto action_dependent_flag_it = parsed_queue_config_it->bool_params_.find("arrival_action_dependent");
                            if (action_dependent_flag_it != parsed_queue_config_it->bool_params_.end())
                            {
                                if(action_dependent_flag_it->second.first)
                                {
                                    new_controller_struct->is_arrival_action_dependent = true;
                                    new_controller_struct->expected_arrival_service_ = 
                                                            nh_.serviceClient<TMetricControlPredictionSrv>(service_name_temp, true);
                                    ROS_INFO_STREAM("Waiting for the server service named :" << new_controller_struct->expected_arrival_service_.getService());
                                    new_controller_struct->expected_arrival_service_.waitForExistence();
                                }
                                else
                                {
                                    // The parameter was not found in the rosparam file
                                    if (!action_dependent_flag_it->second.second)
                                    {
                                        ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing its arrival_action_dependent parameter. Will be set to false.");
                                    }
                                    new_controller_struct->is_arrival_action_dependent = false;
                                    new_controller_struct->arrival_independent_from_action_service_ = 
                                                            nh_.serviceClient<ros_queue_msgs::FloatRequest>(service_name_temp, true);
                                    ROS_INFO_STREAM("Waiting for the server service named :" << new_controller_struct->arrival_independent_from_action_service_.getService());
                                    new_controller_struct->arrival_independent_from_action_service_.waitForExistence();
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
                            
                            // Verify if the departures of the queue is affected by the renewal time.
                            auto departure_renewal_flag_it = parsed_queue_config_it->bool_params_.find("departure_renewal_dependent");
                            if (controller_type_ == ControllerType::RenewalDriftPlusPenalty && 
                                departure_renewal_flag_it != parsed_queue_config_it->bool_params_.end() && 
                                departure_renewal_flag_it->second.second)
                            {
                                new_controller_struct->is_departure_renewal_dependent = departure_renewal_flag_it->second.first;
                            }

                            // Verify if it should subscribe to a srv with an action request.
                            auto action_dependent_flag_it = parsed_queue_config_it->bool_params_.find("departure_action_dependent");
                            if (action_dependent_flag_it != parsed_queue_config_it->bool_params_.end())
                            {
                                if(action_dependent_flag_it->second.first)
                                {
                                    new_controller_struct->is_departure_action_dependent= true;
                                    new_controller_struct->expected_departure_service_ = 
                                                            nh_.serviceClient<TMetricControlPredictionSrv>(service_name_temp, true);
                                    ROS_INFO_STREAM("Waiting for the server service named :" << new_controller_struct->expected_departure_service_.getService());
                                    new_controller_struct->expected_departure_service_.waitForExistence();
                                }
                                else
                                {
                                    // The parameter was not found in the rosparam file
                                    if (!action_dependent_flag_it->second.second)
                                    {
                                        ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing its departure_action_dependent parameter. Will be set to false.");
                                    }
                                    new_controller_struct->is_departure_action_dependent= false;
                                    new_controller_struct->departure_independent_from_action_service_ = 
                                                            nh_.serviceClient<ros_queue_msgs::FloatRequest>(service_name_temp, true);
                                    ROS_INFO_STREAM("Waiting for the server service named :" << new_controller_struct->expected_departure_service_.getService());
                                    new_controller_struct->expected_departure_service_.waitForExistence();
                                    
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

        /**
         * @brief Callback of called from the spin to exectute all the steps from the controller.
        */
        void controllerCallback()
        {
            // Time measurement variables
            std::chrono::time_point<std::chrono::system_clock> time_0 = std::chrono::high_resolution_clock::now();
            std::chrono::time_point<std::chrono::system_clock> time_cursor = time_0;
            double virtual_queues_current_state_update_time_spent = 0.0f;
            double action_set_time_spent = 0.0f;
            double get_parameters_time_spent = 0.0f;
            double compute_optimization_time_spent = 0.0f;
            double send_best_action_time_spent = 0.0f;
            double update_virtual_queues_time_spent = 0.0f; 
            double total_time = 0.0f;
            
            ROS_DEBUG_STREAM("Queue controller: Starting control loop of "<< ros::this_node::getName());
            
            if (inversed_control_and_update_steps_)
            {
                updateVirtualQueuesBasedOnCurrentState();
            }
            time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, virtual_queues_current_state_update_time_spent);

            TPotentialActionSetMsg action_set =  getActionSet();
            time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, action_set_time_spent);

            int nb_actions = action_set.action_set.size();
            if (nb_actions > 0)
            {
                std::vector<ActionParameters> action_parameters_list(nb_actions);
                if(getParametersForControlStep(action_set, action_parameters_list))
                {
                    time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, get_parameters_time_spent);
                    
                    ActionParameters best_action_parameters;
                    if (controller_type_ == ControllerType::DriftPlusPenalty)
                    {
                        best_action_parameters = computeMinDriftPlusPenalty(action_parameters_list);
                    }
                    else if (controller_type_ == ControllerType::RenewalDriftPlusPenalty)
                    {
                        best_action_parameters = computeRenewalMinDriftPlusPenalty(action_parameters_list);
                    }

                    time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, compute_optimization_time_spent);
                
                    sendBestCommand(best_action_parameters.action);
                    time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, send_best_action_time_spent);

                    if (!inversed_control_and_update_steps_)
                    {
                        updateVirtualQueuesBasedOnBestAction(best_action_parameters);
                    }
                    time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, update_virtual_queues_time_spent);
                }
            }

            queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_0, total_time);
            if (controller_time_step_ != 0.0)
            {
                double controller_time_step_ms;
                if (controller_type_ == ControllerType::RenewalDriftPlusPenalty)
                {
                    controller_time_step_ms = min_renewal_time_*1000.0;
                }
                else
                {
                    controller_time_step_ms = controller_time_step_*1000.0;
                }
                ROS_DEBUG_STREAM("Queue controller: Control loop of "<< ros::this_node::getName() << " took " << total_time << " ms to complete. Time spent in each step: \n" 
                                  << "virtual_queues_current_state_update_time_spent: " << virtual_queues_current_state_update_time_spent << " ms (" << virtual_queues_current_state_update_time_spent/total_time*100<< "%), \n" 
                                  << "action_set_time_spent: " << action_set_time_spent << " ms (" << action_set_time_spent/total_time*100<< "%), \n" 
                                  << "get_parameters_time_spent: " << get_parameters_time_spent << " ms (" << get_parameters_time_spent/total_time*100<< "%), \n" 
                                  << "compute_optimization_time_spent: " << compute_optimization_time_spent << " ms (" << compute_optimization_time_spent/total_time*100<< "%), \n"
                                  << "send_best_action_time_spent: " << send_best_action_time_spent << " ms (" << send_best_action_time_spent/total_time*100<< "%), \n"
                                  << "update_virtual_queues_time_spent: " << update_virtual_queues_time_spent << " ms (" << update_virtual_queues_time_spent/total_time*100<< "%), \n"
                                  << "iddle time (based on tmin for renewal):" << controller_time_step_ms - total_time << " ms (" << (controller_time_step_ms - total_time)/controller_time_step_ms*100<< "%)");        
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

            queue_controller_utils::check_persistent_service_connection<TPotentialActionSetSrv>(nh_, solution_space_client_);
            
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
            queue_controller_utils::check_persistent_service_connection<TMetricControlPredictionSrv>(nh_, penalty_service_client_);

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
            queue_controller_utils::check_persistent_service_connection<TMetricControlPredictionSrv>(nh_, renewal_service_client_);
            TMetricControlPredictionSrv renewal_time_msg;
            renewal_time_msg.request.action_set = action_set_msg;
            if(renewal_service_client_.call(renewal_time_msg))
            {
                if(renewal_time_msg.response.predictions.size() != size_of_actions)
                {
                    are_parameters_valid = false;
                    ROS_WARN_STREAM_THROTTLE(2, "Returned renewal time array doesn't contain the same amount of elements has the action set (expected " << size_of_actions << ", received "<< renewal_time_msg.response.predictions.size() <<")");
                }
            }
            else
            {
                are_parameters_valid = false;
                ROS_WARN_STREAM_THROTTLE(2, "Failed to call the renewal time service");
            }

            // Queues services
            for (auto queue_it = queue_list_.begin(); queue_it != queue_list_.end(); ++queue_it)
            {
                const string& queue_name = queue_it->first;
                
                // Arrivals
                if(queue_it->second->is_arrival_action_dependent)
                {
                    TMetricControlPredictionSrv arrival_predictions;
                    arrival_predictions.request.action_set = action_set_msg;

                    queue_controller_utils::check_persistent_service_connection<TMetricControlPredictionSrv>(nh_, queue_it->second->expected_arrival_service_);

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
                else
                {
                    ros_queue_msgs::FloatRequest arrival_prediction;

                    queue_controller_utils::check_persistent_service_connection<ros_queue_msgs::FloatRequest>(nh_, queue_it->second->arrival_independent_from_action_service_);
                    
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

                // Departures
                if(queue_it->second->is_departure_action_dependent)
                {
                    TMetricControlPredictionSrv departure_predictions;
                    departure_predictions.request.action_set = action_set_msg;

                    queue_controller_utils::check_persistent_service_connection<TMetricControlPredictionSrv>(nh_, queue_it->second->expected_departure_service_);

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
                else
                {
                    ros_queue_msgs::FloatRequest departure_prediction;

                    queue_controller_utils::check_persistent_service_connection<ros_queue_msgs::FloatRequest>(nh_, queue_it->second->departure_independent_from_action_service_);

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
            }
            // Populate the output if the parameters are valid
            if (are_parameters_valid)
            {
                for (int action_index = 0; action_index < size_of_actions; ++action_index)
                {
                    action_parameters_output[action_index].penalty = penalty_prediction_srv.response.predictions[action_index];
                    action_parameters_output[action_index].expected_renewal_time = renewal_time_msg.response.predictions[action_index];

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
         * @brief Used the renewal min drift-plus-penalty algorithm to compute the action that minimize the penalty and stabilizes all the queues (if possible).
         * @param action_parameters_list All the parameters and variables for the penalty and the queues for each actions.
         * @return Returns the best action including all the metrics.
        */
        ActionParameters computeRenewalMinDriftPlusPenalty(std::vector<ActionParameters>& action_parameters_list)
        {
            auto best_action = action_parameters_list.begin();

            bool is_first_cost = true;

            for (auto action_parameters_it = action_parameters_list.begin(); action_parameters_it != action_parameters_list.end(); ++action_parameters_it)
            {
                const double& expected_renewal_time = action_parameters_it->expected_renewal_time;

                action_parameters_it->cost = v_parameter_*action_parameters_it->penalty;
                for(auto queue_it = action_parameters_it->queue_parameters.begin(); queue_it != action_parameters_it->queue_parameters.end(); ++queue_it)
                {
                    const string& queue_name = queue_it->first;

                    // Adjusts the arrivals based on the renewal time. The values are directly changed in the object so the manual
                    // changes to the virtual queues will be based on the expected_renewal_time in the later controller steps.
                    if (queue_list_[queue_name]->is_arrival_renewal_dependent)
                    {
                        queue_it->second.expected_arrivals = expected_renewal_time*queue_it->second.expected_arrivals;
                    }
                    if (queue_list_[queue_name]->is_departure_renewal_dependent)
                    {
                        queue_it->second.expected_departures = expected_renewal_time*queue_it->second.expected_departures;
                    }

                    action_parameters_it->cost += queue_list_[queue_name]->weight_ * 
                                                queue_it->second.current_size * 
                                                (queue_it->second.expected_arrivals - queue_it->second.expected_departures);
                }

                // TODO: Verify formulation
                if (is_penalty_renewal_dependent_ && expected_renewal_time > 0.0f)
                {
                    action_parameters_it->cost = action_parameters_it->cost/expected_renewal_time; 
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
            TActionLibOutputGoalType action_lib_output;
            action_lib_output.action_goal = best_action;

            if(best_action_client_)
            {
                best_action_client_->sendGoal(action_lib_output);
            }
        }
        
        /**
         * @brief Send a signal to the queue server to update the virtual queues based on the 
         * current state of the system.
        */
        void updateVirtualQueuesBasedOnCurrentState()
        {
            queue_controller_utils::check_persistent_service_connection<std_srvs::Empty>(nh_, virtual_queues_trigger_);

            std_srvs::Empty trigger;
            if(!virtual_queues_trigger_.call(trigger))
            {
                ROS_WARN_STREAM("Failed to call the virtual queue trigger service");
            }
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
         * @brief Time after which, if the actionlib output did not succeed, the controller will be called.
         * Only used by the renewal_min_drift_plus_penalty controller.
        */
        float max_renewal_time_ = 0.0f;

        /**
         * @brief Time before which if the actionlib output suceed, the controller will wait for the difference
         *  before being called. Only used by the renewal_min_drift_plus_penalty controller.
        */
        float min_renewal_time_ = 0.0f;

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
         * @brief Indicates if the controller should optimize the time average of the ratio between the penalty and the renewal time.
         * Only used by the renewal_min_drift_plus_penalty controller.
        */
        bool is_penalty_renewal_dependent_ = false;

        /**
         * @brief Name of the queue server node linked to the queue_controller. Mainly used as a suffix
         * to solve all the ROS topic and ROS service names provided by the queue server. 
        */
        string queue_server_name_;
};
