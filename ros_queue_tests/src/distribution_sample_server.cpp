#include "ros_queue_tests/distribution_sample_server.hpp"

#include <ros/console.h>

#include "ros_queue_tests/distribution_sample_topic_size.hpp"
#include "ros_queue_tests/inverted_poisson.hpp"

#include "rosparam_utils/xmlrpc_utils.hpp"

DistributionSampleServer::DistributionSampleServer(ros::NodeHandle& nh, float publisher_rate):nh_(nh)
{
    ROS_INFO("Distribution server: Set the logger to Info");

    loadROSParamsAndCreateServices();

    pub_timer_ = nh_.createTimer(ros::Duration(1.0/publisher_rate), &DistributionSampleServer::serverSpin, this);
};

void DistributionSampleServer::serverSpin(const ros::TimerEvent& timer_event)
{
    for (auto it = distribution_sample_publishers_.begin(); it != distribution_sample_publishers_.end(); ++it)
    {
        it->get()->publishFromRamdomSample();
    }
}

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
                                                                distribution_service_param_struct.lambda)||
                               xmlrpc_utils::paramMatchAndParse(parameters[parameter_index],"topic_name",
                                                                distribution_service_param_struct.topic_name)||
                               xmlrpc_utils::paramMatchAndParse(parameters[parameter_index],"type_of_response",
                                                                distribution_service_param_struct.type_of_response)
                                                                ))
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

    if(params.service_name.empty() && params.topic_name.empty())
    {
        ROS_ERROR_STREAM(logging_prefix <<": No service_name or topic_name are defined.");
        is_a_parameter_invalid = true;
    }
    else if(!params.service_name.empty() && !params.topic_name.empty())
    {
        ROS_WARN_STREAM(logging_prefix <<": service_name and topic_name are defined. The topic will be ignored");
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
            
            if (!params.service_name.empty())
            {
                if(params.type_of_response.empty())
                {
                    ROS_ERROR_STREAM(logging_prefix <<": No type_of_response is defined for a service distribution.");
                }
                else if (params.type_of_response == "float")
                {
                    std::unique_ptr<DistributionSampleService<ros_queue_msgs::FloatRequest>> new_sampling_service = std::make_unique<DistributionSampleService<ros_queue_msgs::FloatRequest>>(std::move(new_inverted_poisson),
                                                                                                                                    params.service_name,
                                                                                                                                    nh_);
                    distribution_sample_float_services_.push_back(std::move(new_sampling_service));
                }
                else if (params.type_of_response == "int")
                {
                    std::unique_ptr<DistributionSampleService<ros_queue_msgs::ByteSizeRequest>> new_sampling_service = std::make_unique<DistributionSampleService<ros_queue_msgs::ByteSizeRequest>>(std::move(new_inverted_poisson),
                                                                                                                                    params.service_name,
                                                                                                                                    nh_);
                    distribution_sample_int_services_.push_back(std::move(new_sampling_service));
                }
                else
                {
                    ROS_ERROR_STREAM(logging_prefix <<": Unrecognized type_of_response named "<< params.type_of_response<<". Supported: float and int");
                }
            }
            else if (!params.topic_name.empty())
            {
                std::unique_ptr<DistributionSampleTopicSize> new_sampling_publisher = std::make_unique<DistributionSampleTopicSize>(std::move(new_inverted_poisson),
                                                                                                                                    params.topic_name,
                                                                                                                                    nh_);
                distribution_sample_publishers_.push_back(std::move(new_sampling_publisher));
            }
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