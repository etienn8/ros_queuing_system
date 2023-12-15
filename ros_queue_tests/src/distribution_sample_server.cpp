#include "ros_queue_tests/distribution_sample_server.hpp"

#include <ros/console.h>

#include "ros_queue_tests/inverted_poisson.hpp"

#include "rosparam_utils/xmlrpc_utils.hpp"

DistributionSampleServer::DistributionSampleServer(ros::NodeHandle& nh):nh_(nh)
{
    ROS_INFO("Distribution server: Set the logger to Info");

    loadROSParamsAndCreateServices();
};

void DistributionSampleServer::loadROSParamsAndCreateServices()
{
    XmlRpc::XmlRpcValue distribution_config_list;

    ROS_INFO("CONFIG: Loading distribution server parameters");
    
    if(nh_.getParam("distributions", distribution_config_list))
    {
        if(distribution_config_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int config_index=0; config_index < distribution_config_list.size(); ++config_index)
            {
                XmlRpc::XmlRpcValue& distribution_config =  distribution_config_list[config_index];

                DistributionSampleServer::DistributionServiceParams distribution_service_param_struct;

                if(distribution_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    const string distribution_name = distribution_config.begin()->first;
                    XmlRpc::XmlRpcValue& parameters = distribution_config.begin()->second;

                    if(parameters.getType() == XmlRpc::XmlRpcValue::TypeArray)
                    {
                        for(int parameter_index=0; parameter_index < parameters.size(); ++parameter_index)
                        {
                            if(!(xmlrpc_utils::paramMatchAndParse(parameters[parameter_index],"service_name",
                                                                distribution_service_param_struct.service_name) ||
                               xmlrpc_utils::paramMatchAndParse(parameters[parameter_index],"distribution_type",
                                                                distribution_service_param_struct.distribution_type) ||
                               xmlrpc_utils::paramMatchAndParse(parameters[parameter_index],"lambda",
                                                                distribution_service_param_struct.lambda)))
                            {
                                ROS_WARN_STREAM("CONFIG: Unexpected parameter " << distribution_name <<" in a distribution parameters.");
                            }
                        }
                        
                        checkAndCreateDistributionService(distribution_service_param_struct, distribution_name);
                    }
                }
            }
        }
    }
    else
    {
        ROS_ERROR("Expected a list for distribution sampling services name \"distributions\".");
    }
}

void DistributionSampleServer::checkAndCreateDistributionService(const DistributionServiceParams& params, const string& distribution_config_name )
{
    bool is_a_parameter_invalid = false;

    string logging_prefix = string("CONFIG of " + distribution_config_name +":");

    if(params.service_name.empty())
    {
        ROS_ERROR_STREAM(logging_prefix <<": No service_name is defined.");
        is_a_parameter_invalid = true;
    } 
    
    if(params.distribution_type == "poisson")
    {
        if(params.lambda < 0.0f)
        {
            ROS_ERROR_STREAM(logging_prefix <<": The lambda of the poisson distribution should be defined and positive");
            is_a_parameter_invalid = true;
        }

        if(!is_a_parameter_invalid)
        {
            std::unique_ptr<InversedCumulativeDistribution> new_inverted_poisson = std::make_unique<InvertedPoisson>(params.lambda);
            std::unique_ptr<DistributionSampleService> new_sampling_service = std::make_unique<DistributionSampleService>(std::move(new_inverted_poisson),
                                                                                                                            params.service_name,
                                                                                                                            nh_);
            //new_sampling_service->initROSService();
            distribution_sample_services_.push_back(std::move(new_sampling_service));
        }
    }
    else if(params.distribution_type.empty())
    {
        ROS_ERROR_STREAM(logging_prefix << "No distribution type is defined.");
        is_a_parameter_invalid = true;
    } 
    else
    {
        ROS_ERROR_STREAM(logging_prefix << params.distribution_type <<" is an invalid type of distribution.");
        is_a_parameter_invalid = true;
    }
}