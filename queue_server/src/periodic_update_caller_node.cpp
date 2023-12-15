#include <string>

#include "ros/ros.h"

#include "std_srvs/Empty.h"

using std::string;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "periodic_update_caller");

    ros::NodeHandle nh("~");

    // Fetch parameters from server
    float update_rate=0.0f;
    string trigger_service_name = "";

    if(!nh.getParam("update_rate", update_rate))
    {
        ROS_ERROR("Periodic update caller config: The update_rate param is undefined. Won't create the node.");
        return 0;
    }

    if(!nh.getParam("trigger_service_name", trigger_service_name))
    {
        ROS_ERROR("Periodic update caller config: The trigger_service_name param is undefined. Won't create the node.");
        return 0;
    }

    // Initialize the service
    ros::ServiceClient trigger_client = nh.serviceClient<std_srvs::Empty>(trigger_service_name);
    std_srvs::Empty empty_service_msg;

    // Create the periodic function
    ros::Rate rate(update_rate);

    while(ros::ok())
    {
        trigger_client.call(empty_service_msg);
        
        ros::spinOnce();
        rate.sleep();
    }

}