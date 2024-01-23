#include "queue_controller/queue_controller.hpp"

#include <vector>
#include <utility>
#include <string>

#include "std_srvs/Empty.h"

#include "ros_queue_msgs/MetricPredictions.h"
#include "ros_queue_msgs/PotentialSolutionSpaceFetch.h"
#include "ros_queue_msgs/QueueServerStateFetch.h"
#include "ros_queue_msgs/VirtualQueueChangesList.h"

#include "rosparam_utils/xmlrpc_utils.hpp"
#include "rosparam_utils/parameter_package_fetch_struct.hpp"

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
        penalty_service_client_ = nh_.serviceClient<ros_queue_msgs::MetricPredictions>(penalty_service_name);
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
    
    vector<xmlrpc_utils::ParameterPackageFetchStruct> queue_configs= 
                    xmlrpc_utils::fetchMatchingParametersFromList(nh_, ros::this_node::getName(),
                                                                  "queue_list", fetch_struct);

    // Intialize controller
    if (can_create_controller)
    {   
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