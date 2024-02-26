#include "ros/ros.h"

#include "queue_controller/queue_controller.hpp"
#include "queue_controller/action_configs/transmission_vector_controller_config.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "queue_controller");
    ros::NodeHandle nh("~");
    
    QueueController<METRIC_CONTROL_PREDICTION_SRV, POTENTIAL_ACTION_SET_MSG, POTENTIAL_ACTION_SET_SRV, ACTION_LIB_OUTPUT_TYPE, ACTION_LIB_OUTPUT_GOAL_TYPE> queue_controller(nh);

    // Perform the ros::spin() to process the callbacks and runs the controller.
    queue_controller.spin();
}