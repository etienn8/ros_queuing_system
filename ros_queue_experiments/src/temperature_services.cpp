#include "ros_queue_experiments/metrics/temperature_services.hpp"
#include <string>

#include "ros_queue_experiments/AuvStates.h"
#include "ros_queue_experiments/auv_states.hpp"

using std::string;

TemperatureServices::TemperatureServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services): MetricServices(nh, metric_name, auv_state_manager), nh_(nh), renewal_time_services_(renewal_time_services)
{
    XmlRpc::XmlRpcValue temperature_config;

    if(nh_.getParam("temp", temperature_config))
    {
        for(int model_index =0; model_index < temperature_config.size(); ++model_index)
        {
            auto model_it = temperature_config[model_index].begin();

            XmlRpc::XmlRpcValue temp_prediction_config = model_it->second;

            if(model_it->first == "prediction_model")
            {  
                for(int zone_config_index = 0; zone_config_index < temp_prediction_config.size(); ++zone_config_index)
                {
                    auto zone_config_it = temp_prediction_config[zone_config_index].begin();

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
                        
                        if (value_name == "increase")
                        {
                            arrival_predictions_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                        else if (value_name == "decrease")
                        {
                            departure_predictions_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                    }
                }
            }
        }
    }
}

bool TemperatureServices::realMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    res.value = current_states.temperature;
    return true;
}

bool TemperatureServices::expectedMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    if(renewal_time_services_)
    {
        ros_queue_experiments::AuvStates current_states = getCurrentStates();
        
        for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
            AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);

            float predicted_renewal_time = renewal_time_services_->getPredictedRenewalTimeWithTransitionFromCurrentState(zone);
            
            /**
             * Assume that the temperature of the trajectory is the same as the end zone. Thus we 
             * integrate the change over time as if the change was static over the action.
             */ 
            
            float temperature_change = predicted_renewal_time*(arrival_predictions_[zone] - departure_predictions_[zone]);
            float predicted_temperature = current_states.temperature + temperature_change;

            res.predictions.push_back(predicted_temperature);
        }

        return true;
    }
    return false;
}
