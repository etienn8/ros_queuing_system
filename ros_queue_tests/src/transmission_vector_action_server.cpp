#include "ros/ros.h"

#include <vector>

#include "ros_queue_msgs/TransmissionVector.h"
#include "ros_queue_msgs/PotentialTransmissionVectorSpaceFetch.h"

#include <string>

using std::string;

bool transmission_vector_action_set_callback(ros_queue_msgs::PotentialTransmissionVectorSpaceFetch::Request& req,
                                    ros_queue_msgs::PotentialTransmissionVectorSpaceFetch::Response& res)
{
  ros_queue_msgs::TransmissionVector action;
  action.transmission_vector = std::vector<uint8_t>{1, 1, 0};

  res.action_set.action_set.push_back(action);

  action.transmission_vector = std::vector<uint8_t>{1, 0, 1};

  res.action_set.action_set.push_back(action);

  action.transmission_vector = std::vector<uint8_t>{0, 1, 1};

  res.action_set.action_set.push_back(action);  

  return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv, "transmission_vector_action_server");

    ros::NodeHandle nh("~");

    ros::ServiceServer service = nh.advertiseService("transmission_vector_action_set", &transmission_vector_action_set_callback);

    ros::spin();
}

