#pragma once

#include <map>
#include <memory>

#include "ros/ros.h"

#include "ros_queue_experiments/auv_states.hpp"
#include "ros_queue_experiments/metrics/renewal_time_services.hpp"

#include "dual_metric_services.hpp"

using std::string;

class TemperatureServices: public DualMetricServices
{
    public:
        TemperatureServices(ros::NodeHandle& nh, std::string metric_name, std::shared_ptr<AUVStateManager> auv_state_manager, std::shared_ptr<RenewalTimeServices> renewal_time_services);

        float getRealArrival(AUVStates::Zones zone);
        float getRealDeparture(AUVStates::Zones zone);

        /**
         * @brief Type of internal dynamic model of the temperature.
         */
        enum class TemperatureModel
        {
            Linear =0,      ///< The temperature is modeled as a linear function of time. T = rate*t + T_start
            Differential    ///< The temperature is modeled as a differential equation. T = T_final + (T_final -T_start)*exp(-time/time_constant))
        };

        /**
         * @brief Get the type of the internal dynamic model of the temperature
         * @return Type of the internal dynamic model of the temperature.
         */
        TemperatureModel getTemperatureModel() const;

        /**
         * @brief Computes the new temperature when the robot spends some time in a given zone and initial 
         * temperature based on the real parameters.The evolution depends on the temperature_model_.
         * @param zone Zone where the robot is.
         * @param initial_temperature The temperature at the start of the evaluation frame.
         * @param time Time to which the temperature should be evaluated.
         * @return The new temperature at time.
         */
        float computeRealNewTemperature(AUVStates::Zones zone, float initial_temperature, float time);

        /**
         * @brief Computes the new temperature when the robot spends some time in a given zone and initial 
         * temperature based on the expected parameters.The evolution depends on the temperature_model_.
         * @param zone Zone where the robot is.
         * @param initial_temperature The temperature at the start of the evaluation frame.
         * @param time Time to which the temperature should be evaluated.
         * @return The new temperature at time.
         */
        float computeExpecteNewTemperature(AUVStates::Zones zone, float initial_temperature, float time);

        /**
         * @brief Computes the time integral of the temperature when the robot spends some time in a given zone and initial 
         * temperature based on the real parameters.The evolution depends on the temperature_model_.
         * @param zone Zone where the robot is.
         * @param initial_temperature The temperature at the start of the evaluation frame.
         * @param time Time to which the temperature should be evaluated.
         * @return The integral of the temperature at time.
         */
        float computeRealTimeIntegralNewTemperature(AUVStates::Zones zone, float initial_temperature, float time);

        /**
         * @brief Computes the time integral of the temperature when the robot spends some time in a given zone and initial 
         * temperature based on the expected parameters.The evolution depends on the temperature_model_.
         * @param zone Zone where the robot is.
         * @param initial_temperature The temperature at the start of the evaluation frame.
         * @param time Time to which the temperature should be evaluated.
         * @return The integral of the temperature at time.
         */
        float computeExpectedTimeIntegralNewTemperature(AUVStates::Zones zone, float initial_temperature, float time);

    protected:
        // Temperature specific that could be swapped for inversed limits
        // Change callbacks
        virtual bool TempRealArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                               ros_queue_msgs::FloatRequest::Response& res);

        virtual bool TempRealArrivalPredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);
        
        virtual bool TempExpectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);

        virtual bool TempRealDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                 ros_queue_msgs::FloatRequest::Response& res);
        
        virtual bool TempRealDeparturePredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);

        virtual bool TempExpectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) ;
        
        // Rate callbacks
        virtual bool TempRealArrivalRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res);
        
        virtual bool TempExpectedArrivalRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);

        virtual bool TempRealDepartureRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res);
        
        virtual bool TempExpectedDepartureRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res);

        // Methods to override
        // Change callbacks
        virtual bool realArrivalMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                               ros_queue_msgs::FloatRequest::Response& res) override;

        virtual bool realArrivalPredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;
        
        virtual bool expectedArrivalMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool realDepartureMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                                 ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool realDeparturePredictionMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                                         ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool expectedDepartureMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;
        
        // Rate callbacks
        virtual bool realArrivalRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedArrivalRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        virtual bool realDepartureRateMetricCallback(ros_queue_msgs::FloatRequest::Request& req, 
                                        ros_queue_msgs::FloatRequest::Response& res) override;
        
        virtual bool expectedDepartureRateMetricCallback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req, 
                                            ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res) override;

        float temp_target_ = 0.0f;
    private:
        /**
         * @brief Temperature model type
         */
        TemperatureModel temperature_model_ = TemperatureModel::Linear;

        std::map<AUVStates::Zones, float> expected_arrivals_;
        std::map<AUVStates::Zones, float> expected_departures_;

        std::map<AUVStates::Zones, float> real_expected_arrivals_;
        std::map<AUVStates::Zones, float> real_expected_departures_;

        // Maps for differentical the differential model.
        std::map<AUVStates::Zones, float> real_final_temperatures_;
        std::map<AUVStates::Zones, float> expected_final_temperatures_;
        float time_constant_ = 1.0f;

        // Parameters and variables for the real change service
        ros::Time last_arrival_change_service_call_time_;
        ros::Time last_departure_change_service_call_time_;
        bool is_first_arrival_change_call_ = true;
        bool is_first_departure_change_call_ = true;
        float temperature_at_last_change_call_ = 0.0f;
};