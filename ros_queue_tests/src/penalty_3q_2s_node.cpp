#include "ros/ros.h"

#include "ros_queue_msgs/MetricTransmissionVectorPredictions.h"


bool penalty_evaluation_callback(ros_queue_msgs::MetricTransmissionVectorPredictions::Request& req,
                                 ros_queue_msgs::MetricTransmissionVectorPredictions::Response& res)
{
    for (auto action_it = req.action_set.action_set.begin(); action_it != req.action_set.action_set.end(); ++action_it)
    {
        // The penalty will be simulated as a power cost where the first queue will use 5 units, the second 2 and the third 1.
        res.predictions.push_back(5*action_it->transmission_vector[0] + 
                                  2*action_it->transmission_vector[1] + 
                                  1*action_it->transmission_vector[2]);
    }

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "penalty_3q_2s_node");

    ros::NodeHandle nh("~");

    ros::ServiceServer srv_server = nh.advertiseService("power_penalty", penalty_evaluation_callback);

    ros::spin();
}