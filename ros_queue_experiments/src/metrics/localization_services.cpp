#include "ros_queue_experiments/metrics/localization_services.hpp"

#include <string>

#include "ros_queue_experiments/AuvStates.h"
#include "ros_queue_experiments/auv_states.hpp"

using std::string;

LocalizationServices::LocalizationServices(ros::NodeHandle& nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services): DualMetricServices(nh, metric_name, auv_state_manager, renewal_time_services)
{
    XmlRpc::XmlRpcValue localization_config;
    if(!nh_.getParam("localization_target", localization_target_))
    {
        ROS_ERROR("Localization target is not set");
    }

    if(!nh_.getParam("localization_process_noise", process_noise_))
    {
        ROS_ERROR("Localization noise variance not set");
    }

    if(!nh_.getParam("a_dynamic_", a_dynamic_))
    {
        ROS_ERROR("A matrice value of localization is not set");
    }

    if(nh_.getParam("localization", localization_config))
    {
        for(int model_index =0; model_index < localization_config.size(); ++model_index)
        {
            auto model_it = localization_config[model_index].begin();

            XmlRpc::XmlRpcValue loc_prediction_config = model_it->second;

            for(int zone_config_index = 0; zone_config_index < loc_prediction_config.size(); ++zone_config_index)
            {
                auto zone_config_it = loc_prediction_config[zone_config_index].begin();

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
                    
                    if (value_name == "loc_uncertainty")
                    {
                        if(model_it->first == "prediction_model")
                        {
                            predicted_localization_uncertainties_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                        else if(model_it->first == "real_model")
                        {
                            real_localization_uncertainties_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                    }
                    else if (value_name == "sensor_var")
                    {
                        if(model_it->first == "prediction_model")
                        {
                            predicted_localization_sensor_variance_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                        else if(model_it->first == "real_model")
                        {
                            real_localization_sensor_variance_[zone_from_config] = static_cast<float>(static_cast<double>(value_param->second));
                        }
                    }
                }
            }
        }
    }
}

// Change services
bool LocalizationServices::realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                     ros_queue_msgs::FloatRequest::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    const AUVStates::Zones current_zone = AUVStates::getZoneFromTransmissionVector(current_states.current_zone);
    const AUVStates::Zones last_zone = AUVStates::getZoneFromTransmissionVector(current_states.last_zone);

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

    // Return the change as the integral of the rate of the localization 
    float localization_rate = getRealLocalizationUncertainty(current_zone); 
    res.value = time_since_last_action*localization_rate;

    // Add the change that happened during the controller execution 
    float last_zone_localization_rate = getRealLocalizationUncertainty(last_zone);
    
    res.value += last_zone_localization_rate*elapsed_controller_time;
    last_arrival_change_service_call_time_ = current_time;
    return true;
}

bool LocalizationServices::realArrivalPredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    if(renewal_time_services_)
    {
        for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
            AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);

            float expected_time = renewal_time_services_->getRealRenewalTimeWithTransitionFromCurrentState(zone);

            res.predictions.push_back(real_localization_uncertainties_[zone]*expected_time);
        }
    }

    return true;
}

bool LocalizationServices::expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    if(renewal_time_services_)
    {
        for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
            AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);

            float expected_time = renewal_time_services_->getPredictedRenewalTimeWithTransitionFromCurrentState(zone);

            res.predictions.push_back(predicted_localization_uncertainties_[zone]*expected_time);
        }
    }

    return true;
}

 bool LocalizationServices::realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                        ros_queue_msgs::FloatRequest::Response& res)
{
    const ros::Time current_time = ros::Time::now();
    float time_since_last_change = (current_time - last_departure_change_service_call_time_).toSec();
    
    if (is_first_departure_change_call_)
    {
        time_since_last_change = 0.0;
        is_first_departure_change_call_ = false;
    }

    // Return the change as the integral of the rate of the localization 
    res.value = this->localization_target_*time_since_last_change;

    last_departure_change_service_call_time_ = current_time;

    return true;
}

bool LocalizationServices::realDeparturePredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                                 ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    if(renewal_time_services_)
    {
        for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
            AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);

            res.predictions.push_back(localization_target_);
        }
    }
    return true;
}

bool LocalizationServices::expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    if(renewal_time_services_)
    {
        for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
            AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);

            res.predictions.push_back(localization_target_);
        }
    }
    return true;
}


// Rate services
bool LocalizationServices::realArrivalRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    res.value = current_states.localization;
    return true;
}

bool LocalizationServices::expectedArrivalRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{  
    for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
    {
        ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
        AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);

        res.predictions.push_back(predicted_localization_uncertainties_[zone]);
    }

    return true;
}

 bool LocalizationServices::realDepartureRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    res.value = localization_target_;
    return true;
}

 bool LocalizationServices::expectedDepartureRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
    {
        res.predictions.push_back(localization_target_);
    }
    return true;
}


float LocalizationServices::getRealLocalizationUncertainty(AUVStates::Zones zone)
{
    return real_localization_uncertainties_[zone];
}
