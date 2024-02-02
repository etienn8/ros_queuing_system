#pragma once

#include "ros/ros.h"

#include <vector>
#include <memory>

#include "prediction_service.hpp" 

class PredictionServer
{
    public:
        PredictionServer(ros::NodeHandle& nh);

    private:
        ros::NodeHandle nh_;
        
        std::vector<std::unique_ptr<PredictionService>> service_lists_;
};