#include "ros_queue_experiments/metrics/temperature_services.hpp"
#include <string>

#include "ros_queue_experiments/AuvStates.h"
#include "ros_queue_experiments/auv_states.hpp"

using std::string;

TemperatureServices::TemperatureServices(ros::NodeHandle& nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services): DualMetricServices(nh, metric_name, auv_state_manager, renewal_time_services)
{
    XmlRpc::XmlRpcValue temperature_config;

    if(!nh_.getParam("temp_target", temp_target_))
    {
        ROS_ERROR("Missing temp_target parameter.");
    }

    if(nh_.getParam("temp", temperature_config))
    {
        for(int model_index =0; model_index < temperature_config.size(); ++model_index)
        {
            auto model_it = temperature_config[model_index].begin();

            XmlRpc::XmlRpcValue temp_prediction_config = model_it->second;
              
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
                    if(model_it->first == "prediction_model")
                    {
                        if (value_name == "increase")
                        {
                            expected_arrivals_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                        else if (value_name == "decrease")
                        {
                            expected_departures_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                    }
                    if(model_it->first == "real_model")
                    {
                        if (value_name == "increase")
                        {
                            real_expected_arrivals_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                        else if (value_name == "decrease")
                        {
                            real_expected_departures_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                    }
                }
            }
        }
    }
}

float TemperatureServices::getRealArrival(AUVStates::Zones zone)
{
    return real_expected_arrivals_[zone];
}

float TemperatureServices::getRealDeparture(AUVStates::Zones zone)
{
    return real_expected_departures_[zone];
}

// Change service
bool TemperatureServices::TempRealArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                    ros_queue_msgs::FloatRequest::Response& res)
{
    ros_queue_msgs::GetQueueControllerTiming last_renewal_msg;

    if(real_renewal_service_.call(last_renewal_msg))
    {
        ros_queue_experiments::AuvStates current_states = getCurrentStates();
        /* We truly want the temperature at the end of the last state but the current temperature
           should be a good approximation since the error will be the temp_rate*(~2*service_call_time) */
        float current_temperature = current_states.temperature;
        
        AUVStates::Zones current_zone = AUVStates::getZoneFromTransmissionVector(current_states.current_zone);
        float current_zone_temp_rate = this->real_expected_arrivals_[current_zone] - this->real_expected_departures_[current_zone]; 

        float last_renewal_time = last_renewal_msg.response.timing.renewal_time;
        
        /* Return the equivalent queue change which is the integral of the temperature over time. 
           Since, only the end temperature is avaiblable, the integral is done from that point.*/
        float temperature_at_start_of_frame = current_temperature - current_zone_temp_rate*last_renewal_time;
        res.value = temperature_at_start_of_frame*last_renewal_time + 0.5*current_zone_temp_rate*last_renewal_time*last_renewal_time;

        /* To capture all the changes that happened since the last change to the virtual queues, it is necessary to 
           add the change that happened during the last controller execution. */
        AUVStates::Zones last_zone = AUVStates::getZoneFromTransmissionVector(current_states.last_zone);
        float last_zone_temp_rate = this->real_expected_arrivals_[last_zone] - this->real_expected_departures_[last_zone]; 
        
        float controller_execution_time = last_renewal_msg.response.timing.execution_time;
        res.value += temperature_at_start_of_frame*controller_execution_time - 0.5*last_zone_temp_rate*controller_execution_time*controller_execution_time;
    }
    else
    {
        ROS_ERROR_STREAM("Temperature service couldn't call the last renewal service: "<<real_renewal_service_.getService());
        return false;
    }

    return true;
}

bool TemperatureServices::TempRealArrivalPredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                        ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    if(renewal_time_services_)
    {
        ros_queue_experiments::AuvStates current_states = getCurrentStates();
        
        for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
            AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);

            float predicted_renewal_time = renewal_time_services_->getRealRenewalTimeWithTransitionFromCurrentState(zone);
            
            /**
             * Assume that the temperature of the trajectory is the same as the end zone. Thus we 
             * integrate the temperature over time. integrated_temperature = temp_init*time + 0.5*(a-b)*time^2
             */
            float temperature_diff = real_expected_arrivals_[zone] - real_expected_departures_[zone];
            float integrated_temperature = current_states.temperature*predicted_renewal_time + 0.5*temperature_diff*predicted_renewal_time*predicted_renewal_time;

            res.predictions.push_back(integrated_temperature);
        }

        return true;
    }
    return false;
}

bool TemperatureServices::TempExpectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
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
             * integrate the temperature over time. integrated_temperature = temp_init*time + 0.5*(a-b)*time^2
             */
            float temperature_diff = expected_arrivals_[zone] - expected_departures_[zone];
            float integrated_temperature = current_states.temperature*predicted_renewal_time + 0.5*temperature_diff*predicted_renewal_time*predicted_renewal_time;

            res.predictions.push_back(integrated_temperature);
        }

        return true;
    }
    return false;
}

bool TemperatureServices::TempRealDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                      ros_queue_msgs::FloatRequest::Response& res)
{
    ros_queue_msgs::GetQueueControllerTiming last_renewal_msg;

    if(real_renewal_service_.call(last_renewal_msg))
    {
        float last_renewal_time = last_renewal_msg.response.timing.renewal_time;
        float last_controller_execution_time = last_renewal_msg.response.timing.execution_time;
    
        // Integral of the target over the last renewal time and controller execution time
        res.value = this->temp_target_*(last_renewal_time + last_controller_execution_time);
    }
    else
    {
        ROS_ERROR_STREAM("Temperature service couldn't call the last renewal service: "<<real_renewal_service_.getService());
        return false;
    }

    return true;
}

bool TemperatureServices::TempRealDeparturePredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                          ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    for(int action_index = 0; action_index < req.action_set.action_set.size(); ++action_index)
    {
        ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
        AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);
        
        res.predictions.push_back(temp_target_);
    }
    return true;
}

bool TemperatureServices::TempExpectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                          ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    for(int action_index = 0; action_index < req.action_set.action_set.size(); ++action_index)
    {
        ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
        AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);
        
        res.predictions.push_back(temp_target_);
    }
    return true;
}

// Rate services
bool TemperatureServices::TempRealArrivalRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                        ros_queue_msgs::FloatRequest::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    res.value = current_states.temperature;
    return true;
}

bool TemperatureServices::TempExpectedArrivalRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
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
             * It thus gives the temperature at the end of the action.
             */ 
            
            float temperature_change = predicted_renewal_time*(expected_arrivals_[zone] - expected_departures_[zone]);
            float predicted_temperature = current_states.temperature + temperature_change;

            res.predictions.push_back(predicted_temperature);
        }

        return true;
    }
    return false;
}

bool TemperatureServices::TempRealDepartureRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    res.value = temp_target_;
    return true;
}

bool TemperatureServices::TempExpectedDepartureRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    for(int action_index = 0; action_index < req.action_set.action_set.size(); ++action_index)
    {
        res.predictions.push_back(temp_target_);
    }
    return true;
}

// Overriden methods
bool TemperatureServices::realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res)
{
    return TempRealArrivalMetricCallback(req, res);
}

bool TemperatureServices::realArrivalPredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempRealArrivalPredictionMetricCallback(req, res);
}

bool TemperatureServices::expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempExpectedArrivalMetricCallback(req, res);
}

bool TemperatureServices::realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                            ros_queue_msgs::FloatRequest::Response& res)
{
    return TempRealDepartureMetricCallback(req, res);
}

bool TemperatureServices::realDeparturePredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempRealDeparturePredictionMetricCallback(req, res);
}

bool TemperatureServices::expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempExpectedDepartureMetricCallback(req, res);
}

// Rate callbacks
bool TemperatureServices::realArrivalRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    return TempRealArrivalRateMetricCallback(req, res);
}

bool TemperatureServices::expectedArrivalRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempExpectedArrivalRateMetricCallback(req, res);
}

bool TemperatureServices::realDepartureRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    return TempRealDepartureRateMetricCallback(req, res);
}

bool TemperatureServices::expectedDepartureRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    return TempExpectedDepartureRateMetricCallback(req, res);
}