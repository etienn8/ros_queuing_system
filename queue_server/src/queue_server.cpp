#include "queue_server/queue_server.hpp"

#include <string>
#include <utility>
#include <map>
#include <memory>

#include "ros_queue/ros_virtual_queue.hpp"
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queue/lib_queue/float_compare.hpp"

#include "rosparam_utils/xmlrpc_utils.hpp"

// ROS msgs
#include "ros_queue_msgs/QueueInfo.h"
#include "ros_queue_msgs/QueueServerState.h"

// ROS services
#include "ros_queue_msgs/QueueServerStateFetch.h"

using std::string;


QueueServer::QueueServer(ros::NodeHandle& nh, float spin_rate): nh_(nh)
{
    if (nh_.getParam("queue_server_name", queue_server_name_))
    {
        ROS_INFO_STREAM("Initializing queue server named "<< queue_server_name_);
    }
    else
    {
        ROS_INFO_STREAM("Queue server doesn't a name from the param queue_server_name. Queue serve will be initialized with an empty name.");
    }

    loadQueueParametersAndCreateQueues();

    string update_trigger_service_name="";
    if (nh_.getParam("update_trigger_service_name", update_trigger_service_name) && !update_trigger_service_name.empty())
    {
        queue_server_update_virtual_queues_service = nh_.advertiseService(update_trigger_service_name, 
                                                                         &QueueServer::queueUpdateCallback, this);
    }
    else
    {
        ROS_ERROR_STREAM("Queue server expected an update_trigger_service_name parameter to be defined. The virtual queues can't be updated.");
    }

    string server_state_topic_name="";
    if (nh_.getParam("server_state_topic_name", server_state_topic_name) && !server_state_topic_name.empty())
    {
        queue_server_states_pub_ = nh_.advertise<ros_queue_msgs::QueueServerState>(server_state_topic_name,1000);
    }
    else
    {
        ROS_ERROR_STREAM("Queue server expected a server_state_topic_name parameter to be defined. The server states can't be published.");
    }

    // Create the periodic caller of the serverSpin
    spin_timer_ = nh_.createTimer(ros::Duration(1.0/spin_rate), &QueueServer::serverSpin, this);
}

void QueueServer::loadQueueParametersAndCreateQueues()
{
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
                                if (!(xmlrpc_utils::paramMatchAndParse(parameter, "type_of_queue", queue_param_struct.type_of_queue_) || 
                                      xmlrpc_utils::paramMatchAndParse(parameter, "max_queue_size", queue_param_struct.max_queue_size_) ||
                                      xmlrpc_utils::paramMatchAndParse(parameter, "arrival_evaluation_service_name", queue_param_struct.arrival_evaluation_service_name_)||
                                      xmlrpc_utils::paramMatchAndParse(parameter, "departure_evaluation_service_name", queue_param_struct.departure_evaluation_service_name_)||
                                      xmlrpc_utils::paramMatchAndParse(parameter, "arrival_topic_name", queue_param_struct.arrival_topic_name_)||
                                      xmlrpc_utils::paramMatchAndParse(parameter, "tranmission_topic_name", queue_param_struct.tranmission_topic_name_)||
                                      xmlrpc_utils::paramMatchAndParse(parameter, "transmission_evaluation_service_name", queue_param_struct.transmission_evaluation_service_name_)))
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

void QueueServer::serverSpin(const ros::TimerEvent& event)
{
    // Verify that the publisher was initialized
    if (!queue_server_states_pub_.getTopic().empty())
    {
        publishServerStates();
    }

    transmitRealQueues();
}

void QueueServer::checkAndCreateQueue(QueueParamStruct& queue_param_struct)
{
    bool is_a_parameter_missing = false;

    if (queue_param_struct.queue_name_.empty())
    {
        ROS_ERROR("CONFIG: Queue config is missing the queue name");
        is_a_parameter_missing = true;
    }

    if (float_compare(queue_param_struct.max_queue_size_, 0.0f))
    {
        ROS_ERROR_STREAM("CONFIG: Queue named " << queue_param_struct.queue_name_ <<" is missing its max_queue_size.");
        is_a_parameter_missing = true;
    }
    else if (queue_param_struct.max_queue_size_< 0.0f)
    {
        ROS_ERROR_STREAM("CONFIG: Queue named " << queue_param_struct.queue_name_ <<" was configured with a negative max_queue_size of " << queue_param_struct.max_queue_size_<<". Queues expect a positive max queue size.");
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

            if(queue_param_struct.transmission_evaluation_service_name_.empty())
            {
                ROS_WARN_STREAM("CONFIG: Real queue named " << queue_param_struct.queue_name_ <<" is missing its transmission_evaluation_service_name_.");
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
                                                                .transmission_topic_name = queue_param_struct.tranmission_topic_name_,
                                                                .transmission_evaluation_service_name = queue_param_struct.transmission_evaluation_service_name_
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

bool QueueServer::queueUpdateCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
    // Updates all the inequality constraint virtual queues.
    for(auto it = inequality_constraint_virtual_queues_.begin(); it != inequality_constraint_virtual_queues_.end(); ++it)
    {
        it->second->update();
    }

    // Updates all the equality constraint virtual queues.
    for(auto it = equality_constraint_virtual_queues_.begin(); it != equality_constraint_virtual_queues_.end(); ++it)
    {
        it->second->update();
    }

    return true;
}

void QueueServer::publishServerStates()
{
    ros_queue_msgs::QueueServerState server_state_msg;

    server_state_msg.queue_server_name = queue_server_name_;
    
    appendQueueSizesToMsg(real_queues_, server_state_msg);
    appendQueueSizesToMsg(inequality_constraint_virtual_queues_, server_state_msg);
    appendQueueSizesToMsg(equality_constraint_virtual_queues_, server_state_msg);
    
    queue_server_states_pub_.publish(server_state_msg);
}

void QueueServer::transmitRealQueues()
{
    for(auto it = real_queues_.begin(); it != real_queues_.end(); ++it)
    {
        it->second->transmitBasedOnQoS();
    }
}