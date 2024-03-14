#include "ros_queue_experiments/metrics/renewal_time_services.hpp"


RenewalTimeServices::RenewalTimeServices(ros::NodeHandle nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager): MetricServices(nh, metric_name, auv_state_manager), nh_(nh)
{
    XmlRpc::XmlRpcValue renewal_time_config;

    if(nh_.getParam("renewal_time", renewal_time_config))
    {
        for(int model_index =0; model_index < renewal_time_config.size(); ++model_index)
        {
            auto model_it = renewal_time_config[model_index].begin();

            XmlRpc::XmlRpcValue model_configs = model_it->second;

            bool is_real_model = false;

            if(model_it->first == "real_model")
            {
                is_real_model = true;
            }
            for (int from_zone_index = 0 ; from_zone_index < model_configs.size(); ++from_zone_index)
            {
                auto from_zone_xmlrp_it = model_configs[from_zone_index].begin();

                AUVStates::Zones from_zone;
                if(from_zone_xmlrp_it->first == "fromTaskZone")
                {  
                    from_zone = AUVStates::Zones::TaskZone;
                }
                else if (from_zone_xmlrp_it->first == "fromHighLocZone")
                {
                    from_zone = AUVStates::Zones::HighLocZone;
                }
                else if (from_zone_xmlrp_it->first == "fromColdZone")
                {
                    from_zone = AUVStates::Zones::ColdZone;
                }
                else
                {
                    break;
                }

                XmlRpc::XmlRpcValue from_zone_configs = from_zone_xmlrp_it->second;

                for(int to_zone_index = 0; to_zone_index < from_zone_configs.size(); ++to_zone_index)
                {
                    auto to_zone_config_it = from_zone_configs[to_zone_index].begin();

                    const string& zone_name = to_zone_config_it->first;
                    XmlRpc::XmlRpcValue transition_time = to_zone_config_it->second;

                    AUVStates::Zones to_zone = AUVStates::Zones::TaskZone;

                    if (zone_name == "toTaskZone")
                    {
                        to_zone = AUVStates::Zones::TaskZone;
                    }
                    else if(zone_name == "toHighLocZone")
                    {
                        to_zone = AUVStates::Zones::HighLocZone; 
                    }
                    else if(zone_name == "toColdZone")
                    {
                        to_zone = AUVStates::Zones::ColdZone; 
                    }
                    else
                    {
                        break;
                    }

                    if (is_real_model)
                    {
                        real_renewal_time_transitions_[from_zone][to_zone] = static_cast<float>(static_cast<double>(transition_time)); 
                    }
                    else
                    {
                        predicted_renewal_time_transitions_[from_zone][to_zone] =  static_cast<float>(static_cast<double>(transition_time));
                    }
                }
            }
        }
    }
}

float RenewalTimeServices::getRealRenewalTimeWithStateTransition(AUVStates::Zones from_zone, AUVStates::Zones to_zone)
{
    return real_renewal_time_transitions_[from_zone][to_zone];
}

float RenewalTimeServices::getPredictedRenewalTimeWithStateTransition(AUVStates::Zones from_zone, AUVStates::Zones to_zone)
{
    return predicted_renewal_time_transitions_[from_zone][to_zone];
}

float RenewalTimeServices::getRealRenewalTimeWithTransitionFromCurrentState(AUVStates::Zones to_zone)
{
    if(auv_state_manager_)
    {
        const ros_queue_experiments::AuvStates current_states = auv_state_manager_->getCurrentStates();
        const AUVStates::Zones current_zone = AUVStates::getZoneFromTransmissionVector(current_states.current_zone);

        return getRealRenewalTimeWithStateTransition(current_zone, to_zone);
    }

    return 0.0;
}

float RenewalTimeServices::getPredictedRenewalTimeWithTransitionFromCurrentState(AUVStates::Zones to_zone)
{
    if(auv_state_manager_)
    {
        const ros_queue_experiments::AuvStates current_states = auv_state_manager_->getCurrentStates();
        const AUVStates::Zones current_zone = AUVStates::getZoneFromTransmissionVector(current_states.current_zone);

        return getPredictedRenewalTimeWithStateTransition(current_zone, to_zone);
    }
    
    return 0.0;
}

bool RenewalTimeServices::realMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                        ros_queue_msgs::FloatRequest::Response& res)
{
    // The real metric of how much time it took is given by the controller
    return false;
}

bool RenewalTimeServices::expectedMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    if(auv_state_manager_)
    {
        const ros_queue_experiments::AuvStates current_states = auv_state_manager_->getCurrentStates();
        const AUVStates::Zones current_zone = AUVStates::getZoneFromTransmissionVector(current_states.current_zone);

        for (int action_index = 0; action_index < req.action_set.action_set.size(); ++action_index)
        {
            ros_queue_msgs::TransmissionVector& action = req.action_set.action_set[action_index];
            AUVStates::Zones action_zone = AUVStates::getZoneFromTransmissionVector(action);
            res.predictions.push_back(getPredictedRenewalTimeWithStateTransition(current_zone, action_zone));
        }

        return true;
    }
    return false;
}

