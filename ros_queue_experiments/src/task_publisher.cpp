#include "ros_queue_experiments/metrics/task_publisher.hpp"

#include <string>

#include "ros_queue_experiments/AuvStates.h"
#include "ros_queue_experiments/auv_states.hpp"

#include "ros_queue_msgs/ByteSizeRequest.h"

#include "std_msgs/Int32.h"

using std::string;

TaskPublisher::TaskPublisher(ros::NodeHandle& nh, std::string metric_name, 
                            std::shared_ptr<AUVStateManager> auv_state_manager, 
                            std::shared_ptr<RenewalTimeServices> renewal_time_services): 
                                DualMetricServices(nh, metric_name, auv_state_manager, renewal_time_services)
    {
    XmlRpc::XmlRpcValue tasks_config;
    if(nh_.getParam("task_metrics", tasks_config))
    {
        for(int model_index =0; model_index < tasks_config.size(); ++model_index)
        {
            auto model_it = tasks_config[model_index].begin();

            XmlRpc::XmlRpcValue task_prediction_config = model_it->second;

            for(int zone_config_index = 0; zone_config_index < task_prediction_config.size(); ++zone_config_index)
            {
                auto zone_config_it = task_prediction_config[zone_config_index].begin();

                const string& zone_name = zone_config_it->first;
                XmlRpc::XmlRpcValue zone_values = zone_config_it->second;

                AUVStates::Zones zone_from_config = AUVStates::Zones::TaskZone;

                if (zone_name == "TaskZone")
                {
                    zone_from_config = AUVStates::Zones::TaskZone;
                }
                else if(zone_name == "ColdZone")
                {
                    zone_from_config = AUVStates::Zones::ColdZone; 
                }
                else if(zone_name == "HighLocZone")
                {
                    zone_from_config = AUVStates::Zones::HighLocZone; 
                }
                else
                {
                    break;
                }

                for(int value_index = 0; value_index < zone_values.size(); ++value_index)
                {
                    auto value_param = zone_values[value_index].begin();
                    const string& value_name = value_param->first;
                    
                    if (value_name == "departing_tasks_rate")
                    {
                        if(model_it->first == "prediction_model")
                        {
                            predicted_task_departure_rates_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                        else if(model_it->first == "real_model")
                        {
                            real_task_departure_rates_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                    }
                }
            }
        }
    }

    if(nh_.getParam("arrival_task_per_second", arrival_task_per_second_))
    {
        if(arrival_task_per_second_ > 0.0)
        {
            publisher_ = nh_.advertise<std_msgs::Int32>("incoming_tasks", 10);
            timer_ = nh_.createTimer(ros::Duration(1.0/arrival_task_per_second_), &TaskPublisher::publishTask, this);
        }
        else
        {
            ROS_ERROR("Arrival tasks per second must be greater than 0");
        }
    }
    else
    {
        ROS_ERROR("Arrival tasks per second is not set");
    }

    
    last_transmission_time_ = ros::Time::now();
    qos_transmission_service_ = nh_.advertiseService("qos_transmission",&TaskPublisher::qosTransmissionCallback,this);
}

bool TaskPublisher::qosTransmissionCallback(ros_queue_msgs::ByteSizeRequest::Request& req, 
                                           ros_queue_msgs::ByteSizeRequest::Response& res)
{
    ros::Time current_time = ros::Time::now();
    double time_diff = (current_time - last_transmission_time_).toSec() + accumulated_untransmitted_time_;
    
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    AUVStates::Zones current_zone = AUVStates::getZoneFromTransmissionVector(current_states.current_zone);
    
    if (abs(real_task_departure_rates_[current_zone]) > 1e-6)
    {
        const float second_between_messages = 1.0/real_task_departure_rates_[current_zone];
        while(time_diff >= second_between_messages)
        {
            res.nb_of_bytes += sizeof(std_msgs::Int32);
            time_diff -= second_between_messages;
        }

        accumulated_untransmitted_time_ = time_diff;
    }
    else
    {
        accumulated_untransmitted_time_ = 0.0;
        res.nb_of_bytes = 0;
    }

    last_transmission_time_ = current_time;
    return true;
}

void TaskPublisher::publishTask(const ros::TimerEvent& event)
{
    std_msgs::Int32 task;
    task.data = last_task_id_;
    publisher_.publish(task);
    ++last_task_id_;
}


// Change services
bool TaskPublisher::realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                              ros_queue_msgs::FloatRequest::Response& res)
{
    //TODO add real arrival
    return false;
}

bool TaskPublisher::expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                  ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    if(renewal_time_services_)
    {
        for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            AUVStates::Zones target_zone = AUVStates::getZoneFromTransmissionVector(req.action_set.action_set[action_index]);
            
            float renewal_time = renewal_time_services_->getPredictedRenewalTimeWithTransitionFromCurrentState(target_zone);
            int nb_tasks  = static_cast<int>(arrival_task_per_second_*renewal_time);
            //ROS_WARN_STREAM("Expected arrival float: "<< arrival_task_per_second_*renewal_time<< std::endl<<"Expected arrival int: "<< nb_tasks);
            res.predictions.push_back(nb_tasks*sizeof(std_msgs::Int32));
        }
    }
    else
    {
        ROS_ERROR("Renewal time services not set");
    }

    return true;
}

bool TaskPublisher::realArrivalPredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
       if(renewal_time_services_)
    {
        for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            AUVStates::Zones target_zone = AUVStates::getZoneFromTransmissionVector(req.action_set.action_set[action_index]);
            
            float renewal_time = renewal_time_services_->getRealRenewalTimeWithTransitionFromCurrentState(target_zone);
            int nb_tasks  = static_cast<int>(arrival_task_per_second_*renewal_time);
            //ROS_WARN_STREAM("Expected arrival float: "<< arrival_task_per_second_*renewal_time<< std::endl<<"Expected arrival int: "<< nb_tasks);
            res.predictions.push_back(nb_tasks*sizeof(std_msgs::Int32));
        }
    }
    else
    {
        ROS_ERROR("Renewal time services not set");
    }

    return true; 
}

bool TaskPublisher::realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                ros_queue_msgs::FloatRequest::Response& res)
{
    //TODO add real departure
    return false;
}

bool TaskPublisher::expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    if(renewal_time_services_)
    {
        for(int action_index = 0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            AUVStates::Zones target_zone = AUVStates::getZoneFromTransmissionVector(req.action_set.action_set[action_index]);
                
            float renewal_time = renewal_time_services_->getPredictedRenewalTimeWithTransitionFromCurrentState(target_zone);

            int nb_tasks  = static_cast<int>(predicted_task_departure_rates_[target_zone]*renewal_time);
            res.predictions.push_back(nb_tasks*sizeof(std_msgs::Int32));
        }
    }
    else
    {
        ROS_ERROR("Renewal time services not set");
    }

    return true;
}

bool TaskPublisher::realDeparturePredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
       if(renewal_time_services_)
    {
        for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            AUVStates::Zones target_zone = AUVStates::getZoneFromTransmissionVector(req.action_set.action_set[action_index]);
            
            float renewal_time = renewal_time_services_->getRealRenewalTimeWithTransitionFromCurrentState(target_zone);
            int nb_tasks  = static_cast<int>(real_task_departure_rates_[target_zone]*renewal_time);
            
            res.predictions.push_back(nb_tasks*sizeof(std_msgs::Int32));
        }
    }
    else
    {
        ROS_ERROR("Renewal time services not set");
    }

    return true; 
}

// Rate services
bool TaskPublisher::realArrivalRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                  ros_queue_msgs::FloatRequest::Response& res)
{
    res.value = arrival_task_per_second_*sizeof(std_msgs::Int32);
    return true;
}

bool TaskPublisher::expectedArrivalRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                      ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
    {
        res.predictions.push_back(arrival_task_per_second_*sizeof(std_msgs::Int32));
    }

    return true;
}

bool TaskPublisher::realDepartureRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                    ros_queue_msgs::FloatRequest::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    AUVStates::Zones current_zone = AUVStates::getZoneFromTransmissionVector(current_states.current_zone);
    res.value = real_task_departure_rates_[current_zone]*sizeof(std_msgs::Int32);

    return true;
}

bool TaskPublisher::expectedDepartureRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                        ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    for(int action_index = 0; action_index < req.action_set.action_set.size(); ++action_index)
    {
        AUVStates::Zones target_zone = AUVStates::getZoneFromTransmissionVector(req.action_set.action_set[action_index]);
        
        res.predictions.push_back(predicted_task_departure_rates_[target_zone]*sizeof(std_msgs::Int32));
    }

    return true;
}