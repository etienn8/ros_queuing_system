#include "ros_queue_experiments/metrics/temperature_services.hpp"
#include <string>

#include "ros_queue_experiments/AuvStates.h"
#include "ros_queue_experiments/auv_states.hpp"

using std::string;

TemperatureServices::TemperatureServices(ros::NodeHandle& nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services): DualMetricServices(nh, metric_name, auv_state_manager, renewal_time_services)
{
    XmlRpc::XmlRpcValue temperature_config;

    string temp_model_type_string;
    if(nh_.getParam("temp_model_type", temp_model_type_string))
    {
        if(temp_model_type_string == "linear")
        {
            temperature_model_ = TemperatureModel::Linear;
        }
        else if (temp_model_type_string == "differential")
        {
            temperature_model_ = TemperatureModel::Differential;
        }
        else
        {
            ROS_ERROR("Invalid temperature model type.");
        }
    }

    if(!nh_.getParam("temp_target", temp_target_))
    {
        ROS_ERROR("Missing temp_target parameter.");
    }

    if((temperature_model_ == TemperatureModel::Differential) &&
      (!nh_.getParam("temp_time_constant", time_constant_)))
    {
        ROS_ERROR("Missing temp_time_constant parameter.");
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
                        else if (value_name =="Tout")
                        {
                            expected_final_temperatures_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
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
                        else if (value_name == "Tout")
                        {
                            real_final_temperatures_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                    }
                }
            }
        }
    }
}

TemperatureServices::TemperatureModel TemperatureServices::getTemperatureModel() const
{
    return temperature_model_;
}

float TemperatureServices::getRealArrival(AUVStates::Zones zone)
{
    return real_expected_arrivals_[zone];
}

float TemperatureServices::getRealDeparture(AUVStates::Zones zone)
{
    return real_expected_departures_[zone];
}

float TemperatureServices::computeRealNewTemperature(AUVStates::Zones zone, float initial_temperature, float time)
{
    if(temperature_model_ == TemperatureModel::Linear)
    {
        return initial_temperature + (real_expected_arrivals_[zone] - real_expected_departures_[zone])*time;
    }
    else if(temperature_model_ == TemperatureModel::Differential)
    {
        return real_final_temperatures_[zone] - (real_final_temperatures_[zone] - initial_temperature)*std::exp(-time/time_constant_);
    }
}

float TemperatureServices::computeExpecteNewTemperature(AUVStates::Zones zone, float initial_temperature, float time)
{
    if(temperature_model_ == TemperatureModel::Linear)
    {
        return initial_temperature + (expected_arrivals_[zone] - expected_departures_[zone])*time;
    }
    else if(temperature_model_ == TemperatureModel::Differential)
    {
        return expected_final_temperatures_[zone] - (expected_final_temperatures_[zone] - initial_temperature)*std::exp(-time/time_constant_);
    }
}

float TemperatureServices::computeRealTimeIntegralNewTemperature(AUVStates::Zones zone, float initial_temperature, float time)
{
    if(temperature_model_ == TemperatureModel::Linear)
    {
        return initial_temperature*time + 0.5*(real_expected_arrivals_[zone] - real_expected_departures_[zone])*time*time;
    }
    else if(temperature_model_ == TemperatureModel::Differential)
    {
        return real_final_temperatures_[zone]*time + time_constant_*(real_final_temperatures_[zone] - initial_temperature)*(std::exp(-time/time_constant_)-1); 
    }
}

float TemperatureServices::computeExpectedTimeIntegralNewTemperature(AUVStates::Zones zone, float initial_temperature, float time)
{
    if(temperature_model_ == TemperatureModel::Linear)
    {
        return initial_temperature*time + 0.5*(expected_arrivals_[zone] - expected_departures_[zone])*time*time;
    }
    else if(temperature_model_ == TemperatureModel::Differential)
    {
        return expected_final_temperatures_[zone]*time + time_constant_*(expected_final_temperatures_[zone] - initial_temperature)*(std::exp(-time/time_constant_)-1);
    }
}

// Change service
bool TemperatureServices::TempRealArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                    ros_queue_msgs::FloatRequest::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    // Compute the time since the last action
    const ros::Time current_time = ros::Time::now();
    float time_since_last_action = (current_time - current_states.last_transition_time).toSec();
    // Compute the time between the last action and the last virtual queue update which represents the controller execution time.
    float elapsed_controller_time = (current_states.last_transition_time - last_arrival_change_service_call_time_).toSec();

    if(is_first_arrival_change_call_)
    {
        time_since_last_action = 0.0;
        elapsed_controller_time = 0.0;
        is_first_arrival_change_call_ = false;
    }
    // Current zone that is being used since the last action
    const AUVStates::Zones current_zone = AUVStates::getZoneFromTransmissionVector(current_states.current_zone);
    // Zone in which the robot was during the computation of the last controller execution
    const AUVStates::Zones last_zone = AUVStates::getZoneFromTransmissionVector(current_states.last_zone);

    // Compute the integral of temperature that happend during the last controller execution
    const float integral_temperature_controller = computeRealTimeIntegralNewTemperature(last_zone, temperature_at_last_change_call_, elapsed_controller_time);
    
    // Compute the integral of temperature that happened since the last action
    const float temperature_at_start_of_frame = current_states.temperature_last_end_frame;
    const float integral_temperature_action = computeRealTimeIntegralNewTemperature(current_zone, temperature_at_start_of_frame, time_since_last_action);

    res.value = integral_temperature_controller + integral_temperature_action;
    
    /* Update the internal parameters for next call */
    // The current temperature is the temperature at the beginning of the controller execution.
    temperature_at_last_change_call_ = current_states.temperature;
    last_arrival_change_service_call_time_ = current_time;
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
            res.predictions.push_back(computeRealTimeIntegralNewTemperature(zone, current_states.temperature, predicted_renewal_time));
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
             * integrate the temperature over time.
             */
            float integrated_temperature = computeExpectedTimeIntegralNewTemperature(zone, current_states.temperature, predicted_renewal_time);

            res.predictions.push_back(integrated_temperature);
        }

        return true;
    }
    return false;
}

bool TemperatureServices::TempRealDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                      ros_queue_msgs::FloatRequest::Response& res)
{
    const ros::Time current_time = ros::Time::now();
    float time_since_last_change = (current_time-last_departure_change_service_call_time_).toSec();
    if (is_first_departure_change_call_)
    {
        time_since_last_change = 0.0;
        is_first_departure_change_call_ = false;
    }

    // Integral of the target over the last renewal time and controller execution time
    res.value = this->temp_target_*(time_since_last_change);
    last_departure_change_service_call_time_ = current_time;

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
            
            // Integrate the signal to get the change in the queue
            res.predictions.push_back(computeExpecteNewTemperature(zone, current_states.temperature, predicted_renewal_time));
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