#include "queue_server/queue_server.hpp"

#include <string>
#include <utility>
#include <map>
#include <memory>

#include "ros_queue/ros_virtual_queue.hpp"
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queue/lib_queue/float_compare.hpp"

#include "ros_queue_msgs/QueueInfo.h"

using std::string;

template<>
bool QueueServer::paramMatchAndParse<string>(const XmlRpc::XmlRpcValue parameter, const string& parameter_name_to_check_against, string& output)
{
    auto parameter_it = parameter.begin();

    const string& parameter_name = parameter_it->first;
    const XmlRpc::XmlRpcValue& parameter_value = parameter_it->second;
    
    if (parameter_name == parameter_name_to_check_against && parameter_value.getType() ==
        XmlRpc::XmlRpcValue::TypeString)
    {
        output= static_cast<string>(parameter_value);
        return true;
    }
    else
    {
        return false;
    }
}

template<>
bool QueueServer::paramMatchAndParse<float>(const XmlRpc::XmlRpcValue parameter, const string& parameter_name_to_check_against, float& output)
{
    auto parameter_it = parameter.begin();

    const string& parameter_name = parameter_it->first;
    const XmlRpc::XmlRpcValue& parameter_value = parameter_it->second;
    
    if (parameter_name == parameter_name_to_check_against && parameter_value.getType() ==
        XmlRpc::XmlRpcValue::TypeDouble)
    {
        output  = static_cast<float>(static_cast<double>(parameter_value));
        return true;
    }
    else
    {
        return false;
    }
}

QueueServer::QueueServer(ros::NodeHandle& nh): nh_(nh)
{
    if (nh_.getParam("queue_server_name", queue_server_name_))
    {
        ROS_INFO_STREAM("Initializing queue server named "<< queue_server_name_);
    }

    // Initialization of queues based on a config file
    XmlRpc::XmlRpcValue queue_list;

    if(nh_.getParam("queue_list", queue_list))
    {
        if(queue_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for(int queue_index = 0; queue_index < queue_list.size(); ++queue_index)
            {
                XmlRpc::XmlRpcValue queue = queue_list[queue_index];

                if(queue.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    for (auto queue_iterator = queue.begin(); queue_iterator != queue.end(); ++queue_iterator) 
                    {
                        std::string queue_name = queue_iterator->first;
                        XmlRpc::XmlRpcValue& queue_parameters = queue_iterator->second;

                        QueueParamStruct queue_param_struct;
                        queue_param_struct.queue_name_ = queue_name;

                        for(int index_param = 0; index_param < queue_parameters.size(); ++index_param)
                        {
                            XmlRpc::XmlRpcValue parameter = queue_parameters[index_param];
                            if(parameter.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                            {
                                if (!(paramMatchAndParse(parameter, "type_of_queue", queue_param_struct.type_of_queue_) || 
                                      paramMatchAndParse(parameter, "max_queue_size", queue_param_struct.max_queue_size_) ||
                                      paramMatchAndParse(parameter, "arrival_evaluation_service_name", queue_param_struct.arrival_evaluation_service_name_)||
                                      paramMatchAndParse(parameter, "departure_evaluation_service_name", queue_param_struct.departure_evaluation_service_name_)||
                                      paramMatchAndParse(parameter, "arrival_topic_name", queue_param_struct.arrival_topic_name_)||
                                      paramMatchAndParse(parameter, "tranmission_topic_name", queue_param_struct.tranmission_topic_name_)))
                                {
                                    string parameter_name = parameter.begin()->first;

                                    ROS_WARN_STREAM("CONFIG: Unknown parameter in queue_server: " << parameter_name<< ". Will be ignored.");
                                }
                            }
                            else
                            {
                                ROS_WARN_STREAM("CONFIG: A queue parameter is either empty or not a struct.");
                            }
                        }

                        checkAndCreateQueue(queue_param_struct);
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("CONFIG: A queue config in the list is not a structure.");
                }
            }
        }
        else
        {
            ROS_ERROR_STREAM("CONFIG: The queue list not an array.");
        }
    }
    else
    {
            ROS_ERROR_STREAM("CONFIG: Can't find queue list in configuration file.");
    }
}

void QueueServer::addInequalityConstraintVirtualQueue(std::unique_ptr<ROSInConVirtualQueue>&& new_queue)
{ 
    // Emplace the new queue into the map
    inequality_constraint_virtual_queues_.emplace(new_queue->info_.queue_name, std::move(new_queue));
}

void QueueServer::addEqualityConstraintVirtualQueue(std::unique_ptr<ROSEqConVirtualQueue>&& new_queue)
{
    // Emplace the new queue into the map
    equality_constraint_virtual_queues_.emplace(new_queue->info_.queue_name, std::move(new_queue));
}

void QueueServer::addRealQueue(std::unique_ptr<ROSByteConvertedQueue>&& new_queue)
{
    // Emplace the new queue into the map
    real_queues_.emplace(new_queue->info_.queue_name, std::move(new_queue));
}

void QueueServer::checkAndCreateQueue(QueueParamStruct& queue_param_struct)
{
    bool is_a_parameter_missing = false;

    if (queue_param_struct.queue_name_.empty())
    {
        ROS_ERROR("CONFIG: Queue config is missing the queue name");
        is_a_parameter_missing = true;
    }

    if (float_compare(queue_param_struct.max_queue_size_, -1.0f))
    {
        ROS_ERROR_STREAM("CONFIG: Queue named " << queue_param_struct.queue_name_ <<" is missing its max_queue_size.");
        is_a_parameter_missing = true;
    }
    
    if (queue_param_struct.type_of_queue_.empty())
    {
        ROS_ERROR_STREAM("CONFIG: Queue named " << queue_param_struct.queue_name_ <<" is missing its queue_type.");
        is_a_parameter_missing = true;
    }
    else
    {
        // Verify if the type exists
        if(queue_param_struct.type_of_queue_ == "real_queue")
        {
            if(queue_param_struct.arrival_topic_name_.empty())
            {
                ROS_ERROR_STREAM("CONFIG: Real queue named " << queue_param_struct.queue_name_ <<" is missing its arrival_topic_name.");
                is_a_parameter_missing = true;
            }

            if(queue_param_struct.tranmission_topic_name_.empty())
            {
                ROS_ERROR_STREAM("CONFIG: Real queue named " << queue_param_struct.queue_name_ <<" is missing its tranmission_topic_name_.");
                is_a_parameter_missing = true;
            }

            if (!is_a_parameter_missing)
            {
                ROS_INFO_STREAM("Creating real queue: " << queue_param_struct.queue_name_);
                ros_queue_msgs::QueueInfo info;
                info.is_virtual = false;
                info.queue_name = queue_param_struct.queue_name_;
                info.queue_type = queue_param_struct.type_of_queue_;

                std::unique_ptr<ROSByteConvertedQueue> new_queue = 
                    std::make_unique<ROSByteConvertedQueue>((int)queue_param_struct.max_queue_size_, 
                                                            std::move(info),
                                                            nh_,
                                                            (struct ROSByteConvertedQueue::InterfacesArgs)
                                                            {
                                                                .arrival_topic_name = queue_param_struct.arrival_topic_name_,
                                                                .transmission_topic_name = queue_param_struct.tranmission_topic_name_
                                                            });

                addRealQueue(std::move(new_queue));
            }
            else
            {
                ROS_ERROR_STREAM("Will no create the real queue " << queue_param_struct.queue_name_ << " since at least one of its parameter is missing.");
            }
        }
        else if (queue_param_struct.type_of_queue_ == "inequality_constraint_virtual_queue")
        {
            if(queue_param_struct.arrival_evaluation_service_name_.empty())
            {
                ROS_ERROR_STREAM("CONFIG: Virtual queue named " << queue_param_struct.queue_name_ <<" is missing its arrival_evaluation_service_name.");
                is_a_parameter_missing = true;
            }

            if(queue_param_struct.departure_evaluation_service_name_.empty())
            {
                ROS_ERROR_STREAM("CONFIG: Virtual queue named " << queue_param_struct.queue_name_ <<" is missing its departure_evaluation_service_name.");
                is_a_parameter_missing = true;
            }

            if (!is_a_parameter_missing)
            {
                ROS_INFO_STREAM("Creating virtual real queue: " << queue_param_struct.queue_name_);
                ros_queue_msgs::QueueInfo info;
                info.is_virtual = true;
                info.queue_name = queue_param_struct.queue_name_;
                info.queue_type = queue_param_struct.type_of_queue_;

                std::unique_ptr<ROSInConVirtualQueue> new_queue = 
                    std::make_unique<ROSInConVirtualQueue>((int)queue_param_struct.max_queue_size_, 
                                                            std::move(info),
                                                            nh_,
                                                            (struct ROSInConVirtualQueue::InterfacesArgs)
                                                            {
                                                                .arrival_evaluation_service_name = queue_param_struct.arrival_evaluation_service_name_,
                                                                .departure_evaluation_service_name = queue_param_struct.departure_evaluation_service_name_
                                                            });

                addInequalityConstraintVirtualQueue(std::move(new_queue));
            }
            else
            {
                ROS_ERROR_STREAM("Will no create the virtual queue " << queue_param_struct.queue_name_ << " since at least one of its parameter is missing.");
            }
        }
        else if (queue_param_struct.type_of_queue_ == "equality_constraint_virtual_queue")
        {
            if(queue_param_struct.arrival_evaluation_service_name_.empty())
            {
                ROS_ERROR_STREAM("CONFIG: Virtual queue named " << queue_param_struct.queue_name_ <<" is missing its arrival_evaluation_service_name.");
                is_a_parameter_missing = true;
            }

            if(queue_param_struct.departure_evaluation_service_name_.empty())
            {
                ROS_ERROR_STREAM("CONFIG: Virtual queue named " << queue_param_struct.queue_name_ <<" is missing its departure_evaluation_service_name.");
                is_a_parameter_missing = true;
            }

            if (!is_a_parameter_missing)
            {
                ROS_INFO_STREAM("Creating virtual real queue: " << queue_param_struct.queue_name_);
                ros_queue_msgs::QueueInfo info;
                info.is_virtual = true;
                info.queue_name = queue_param_struct.queue_name_;
                info.queue_type = queue_param_struct.type_of_queue_;

                std::unique_ptr<ROSEqConVirtualQueue> new_queue = 
                    std::make_unique<ROSEqConVirtualQueue>((int)queue_param_struct.max_queue_size_, 
                                                            std::move(info),
                                                            nh_,
                                                            (struct ROSEqConVirtualQueue::InterfacesArgs)
                                                            {
                                                                .arrival_evaluation_service_name = queue_param_struct.arrival_evaluation_service_name_,
                                                                .departure_evaluation_service_name = queue_param_struct.departure_evaluation_service_name_
                                                            });

                addEqualityConstraintVirtualQueue(std::move(new_queue));
            }
            else
            {
                ROS_ERROR_STREAM("Will no create the virtual queue " << queue_param_struct.queue_name_ << " since at least one of its parameter is missing.");
            }
        }
        else
        {
            ROS_ERROR_STREAM("CONFIG: Queue named " << queue_param_struct.queue_name_ <<" has a queue type " << 
            queue_param_struct.type_of_queue_ << ", which is not supported.");
            is_a_parameter_missing = true;
        }
    }
}