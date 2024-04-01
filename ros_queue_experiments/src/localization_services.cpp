#include "ros_queue_experiments/metrics/localization_services.hpp"

#include <string>

#include "ros_queue_experiments/AuvStates.h"
#include "ros_queue_experiments/auv_states.hpp"

using std::string;

LocalizationServices::LocalizationServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager): DualMetricServices(nh, metric_name, auv_state_manager), nh_(nh)
{
    XmlRpc::XmlRpcValue localization_config;
    if(!nh_.getParam("localization_target", localization_target_))
    {
        ROS_ERROR("Localization target is not set");
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
                }
            }
        }
    }
}

bool LocalizationServices::realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    res.value = current_states.localization;
    return true;
}

bool LocalizationServices::expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                    ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    ros_queue_experiments::AuvStates current_states = getCurrentStates();
    
    for(int action_index =0; action_index < req.action_set.action_set.size(); ++action_index)
    {
        ros_queue_msgs::TransmissionVector &action = req.action_set.action_set[action_index];
        AUVStates::Zones zone = AUVStates::getZoneFromTransmissionVector(action);

        res.predictions.push_back(predicted_localization_uncertainties_[zone]);
    }

    return true;
}

 bool LocalizationServices::realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                ros_queue_msgs::FloatRequest::Response& res)
{
    res.value = localization_target_;
    return true;
}

 bool LocalizationServices::expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
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
