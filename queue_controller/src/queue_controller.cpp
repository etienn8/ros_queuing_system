#include "queue_controller/queue_controller.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "std_srvs/Empty.h"

#include "ros_queue_msgs/MetricControlPredictions.h"
#include "ros_queue_msgs/PotentialSolutionSpaceFetch.h"
#include "ros_queue_msgs/QueueServerStateFetch.h"
#include "ros_queue_msgs/VirtualQueueChangesList.h"

#include "rosparam_utils/xmlrpc_utils.hpp"
#include "rosparam_utils/parameter_package_fetch_struct.hpp"

#include "ros_queue_msgs/QueueInfoFetch.h"


#include "queue_server/queue_server_utils.hpp"

using std::string;

QueueController::QueueController(ros::NodeHandle& nh): nh_(nh)
{
    bool can_create_controller = true;

    if(nh.getParam("controller_type", controller_type_))
    {
        if (!(controller_type_ == "min_drift_plus_penalty" || controller_type_ == "renewal_min_drift_plus_penalty"))
        {
           can_create_controller = false;
           ROS_ERROR_STREAM("Unknown controller type:" << controller_type_ <<". Supported types: min_drift_plus_penalty and renewal_min_drift_plus_penalty"); 
        }
    }
    else
    {
        can_create_controller = false;
        ROS_ERROR_STREAM("Missing the controller type");
    }

    if (controller_type_ == "min_drift_plus_penalty")
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
        solution_space_client_ = nh_.serviceClient<ros_queue_msgs::PotentialSolutionSpaceFetch>(solution_space_service_name);
    }
    else
    {
        can_create_controller = false;
        ROS_ERROR_STREAM("Missing the solution_space_service_name.");
    }

    string penalty_service_name;
    if(nh.getParam("penalty_service_name", penalty_service_name))
    {
        penalty_service_client_ = nh_.serviceClient<ros_queue_msgs::MetricControlPredictions>(penalty_service_name);
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
    const xmlrpc_utils::ParameterPackageFetchStruct fetch_struct(queue_string_parameter_names_,
                                                            queue_float_parameter_names_);
    
    vector<xmlrpc_utils::ParameterPackageFetchStruct> parsed_queue_configs= 
                    xmlrpc_utils::fetchMatchingParametersFromList(nh_, ros::this_node::getName(),
                                                                  "queue_list", fetch_struct);

    // Intialize controller
    if (can_create_controller)
    {   
        // Transfer queue_configs in internal queue structs
        populateQueueStructures(parsed_queue_configs);

        // Connect to queue_server for queue sizes
        server_state_client_ = nh_.serviceClient<ros_queue_msgs::QueueServerStateFetch>(queue_server_name_ + "/get_server_state");

        // Connect to queue_server for queue server udpates (depends on steps order)
        if (!inversed_control_and_update_steps_)
        {   
            manual_virtual_queue_changes_ = nh_.advertise<ros_queue_msgs::VirtualQueueChangesList>(queue_server_name_ + "/virtual_queue_manual_changes", 1000);
        }
        else
        {
            virtual_queues_trigger_ = nh_.serviceClient<std_srvs::Empty>(queue_server_name_ + "/trigger_service");
        }
        
        // Connect to queue services

        // Create ouptut topic

        // Initialize the triggers
        is_initialized_ = true;
    }
}

bool QueueController::populateQueueStructures(vector<xmlrpc_utils::ParameterPackageFetchStruct> parsed_queue_configs)
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
                    
                    new_controller_struct->expected_arrival_service_ = 
                                            nh_.serviceClient<ros_queue_msgs::MetricControlPredictions>(service_name_temp);
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
                    
                    new_controller_struct->expected_departure_service_ = 
                                            nh_.serviceClient<ros_queue_msgs::MetricControlPredictions>(service_name_temp);
                }
                else
                {
                    is_queue_parameters_ok = false;
                    ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing his expected_departure_service_name parameter");
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
