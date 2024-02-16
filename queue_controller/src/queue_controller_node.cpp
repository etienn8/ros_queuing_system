#include "ros/ros.h"

#include "queue_controller/queue_controller.hpp"
#include "queue_controller/action_configs/bernoulli_action_controller_config.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "queue_controller");
    ros::NodeHandle nh("~");
    
    QueueController<METRIC_CONTROL_PREDICTION_SRV, POTENTIAL_ACTION_SET_MSG, POTENTIAL_ACTION_SET_SRV> queue_controller(nh);

    ros::spin();
}