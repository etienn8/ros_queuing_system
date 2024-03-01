#include "ros_queue_tests/prediction_server.hpp"

#include <string>
#include <memory>

#include "rosparam_utils/xmlrpc_utils.hpp"

using std::string;

PredictionServer::PredictionServer(ros::NodeHandle& nh): nh_(nh)
{
    xmlrpc_utils::ParameterPackageFetchStruct::OptionsStruct fetch_options;
    fetch_options.string_parameter_name_list = std::vector<string>{"control_action_type","distribution_type"};
    fetch_options.float_parameter_name_list = std::vector<string>{"transmission_value"};
    fetch_options.int_parameter_name_list = std::vector<string>{"transmission_vector_id"};
    
    xmlrpc_utils::ParameterPackageFetchStruct parameter_patterns(fetch_options);
    auto prediction_structs = xmlrpc_utils::fetchMatchingParametersFromList(nh_, ros::this_node::getName(),
                                                  "prediction_service_list", parameter_patterns);
    
    for (auto package_struct_it = prediction_structs.begin(); package_struct_it != prediction_structs.end(); ++package_struct_it)
    {
        PredictionService::ParameterOptions service_options;
        
        service_options.service_name = package_struct_it->param_package_name_;

        /**
         * The ..._params_.[].second indicates if the variable was parsed or not and
         * the ..._params_.[].fist is the value of the paramter
        */ 
        
        // Get the strings
        if (package_struct_it->string_params_["control_action_type"].second)
        {
            service_options.control_action_type = package_struct_it->string_params_["control_action_type"].first;
        }

        if (package_struct_it->string_params_["distribution_type"].second)
        {
            service_options.distribution_type = package_struct_it->string_params_["distribution_type"].first;
        }

        // Get the floats
        if (package_struct_it->float_params_["transmission_value"].second)
        {
            service_options.transmission_value = package_struct_it->float_params_["transmission_value"].first;
        }

        // Get the int
        if (package_struct_it->int_params_["transmission_vector_id"].second)
        {
            service_options.is_transmission_evaluation = true;
            service_options.transmission_vector_id = package_struct_it->int_params_["transmission_vector_id"].first;
        }
        
        service_lists_.push_back(std::make_unique<PredictionService>(nh_, service_options));  
    }
}
