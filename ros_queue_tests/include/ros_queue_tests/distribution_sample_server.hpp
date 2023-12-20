#pragma once

#include <string>

#include "ros/ros.h"

#include "distribution_sample_service.hpp"
#include "distribution_sample_topic_size.hpp"

using std::string;

class DistributionSampleServer
{
    public:
        DistributionSampleServer(ros::NodeHandle& nh);

        /**
         * @brief Publishes all generated samples that need to send over topics.  
         */
        void serverSpin();

    private:
        ros::NodeHandle nh_;

        /**
         * @brief Data structure of all parameters that could be usefull to define a distribution. 
         * Used internaly to fetch parameters.
         * @param service_name Name of the ROS service that the given random distribution sampling service.
         * @param distribution_type Type of the distribution from which the ramdom value will be compute from. 
         * Supported type: poisson.
         * @param lambda Lambda parameter of a poisson distribution. 
        */
        struct DistributionServiceParams
        {
            string service_name ="";
            string distribution_type="";
            float lambda=-1.0f;
            string topic_name ="";
        };

        /**
         * @brief Loads the rosparams from "distributions" list param to create distribution sampling service base on the given parameters.
         * Create the service in itself afterward.
        */
        void loadROSParamsAndCreateServices();

        /**
         * @brief Verify that all the expected parameters for the specified distribution are valid and create the service object based on the parameters
         * @param params Parameter structure that contains the possible params that could be use to configure a distribution sampling service.
         * @param distribution_config_name Name of the group of parameters. If used with loadROSParamsAndCreateServices(), it's the name of 
         * distribution in the "distributions" list. It's only used for logging.
        */
        void checkAndCreateDistributionService(const DistributionServiceParams& params, const string& distribution_config_name);

        /**
         * @brief Vectors of all the service objects that hold a ROS service that send a sample of given distribtion function.
        */
        std::vector<std::unique_ptr<DistributionSampleService>> distribution_sample_services_;

        std::vector<std::unique_ptr<DistributionSampleTopicSize>> distribution_sample_publishers_;
};