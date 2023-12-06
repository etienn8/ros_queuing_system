#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "ros/ros.h"

#include "ros_queue/ros_byte_converted_queue.hpp"
#include "ros_queue/ros_virtual_queue.hpp"
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"

#include "ros_queue_msgs/QueueStates.h"
#include "ros_queue_msgs/QueueStatesPrediction.h"

using std::string;

/**
 * @brief Class that creates and stores ros_queues based on a config file, manages the update of the virtual queues, 
 * provides services and topics to inspect the size of the queues and periodically transmits the real queues.
*/
class QueueServer
{
    public:
        /**
         * @brief Constructor that loads the ROS parameters to define all the internal meta values 
         * of the queue server and to create and define all the queues.  
        */
        QueueServer(ros::NodeHandle& nh);
        
        /**
         * @brief Add a virtual queue in the inequality constraint queue map.
         * @param new_queue Rvalue of a unique pointer of a new inequality constraint queue 
         * to move to the inequality constraint map. 
        */
        void addInequalityConstraintVirtualQueue(std::unique_ptr<ROSInConVirtualQueue>&& new_queue);

        /**
         * @brief Add a virtual queue in the equality constraint queue map.
         * @param new_queue Rvalue of a unique pointer of a new equality constraint queue 
         * to move to the equality constraint map. 
        */
        void addEqualityConstraintVirtualQueue(std::unique_ptr<ROSEqConVirtualQueue>&& new_queue);

        /**
         * @brief Add a queue of real data with the queue size in bytes in the real queue map.
         * @param new_queue Rvalue of a unique pointer of a new real queue to move to the real_queue map.
        */
        void addRealQueue(std::unique_ptr<ROSByteConvertedQueue>&& new_queue);

    private:

        /**
         * @brief Structure used internaly to hold temporarily all the possible param that can be used for the queue configuration.
         * The default value of each variable is also used as a check to validate if the variable was set or not.
         * 
        */
        struct QueueParamStruct
        {
            string queue_name_ = "";
            string type_of_queue_ = "";
            float max_queue_size_ = -1.0f;

            string arrival_evaluation_service_name_ = "";
            string departure_evaluation_service_name_ = "";

            string arrival_topic_name_ = "";
            string tranmission_topic_name_ = "";
        };

        /**
         * @brief Checks if the parameter's name matches a given parameter name and parses the value of the parameter 
         * in the output if its the case.
         * @param parameter Struct parameter from a config that contains a name and a XmlRpcValue.
         * @param parameter_name_to_check_against String to match the parameter name against.
         * @param output Reference to an output that is set to the parameter's value if their is a match. Not modify otherwise.
         * @tparam Type of the variable to parse from the parameter.
        */
        template<typename T>
        bool paramMatchAndParse(const XmlRpc::XmlRpcValue parameter, const string& parameter_name_to_check_against, T& output);

        /**
         * @brief Checks if the required parameters for a given queue type are defined,
         *  logs if a parameter is missing and creates a queue in its respective map if they are all defined.
         * @param queue_param_struct Structure that contains all the possible queue configs.
        */
        void checkAndCreateQueue(QueueParamStruct& queue_param_struct);

        /**
         * @brief Map of inequality constraint virtual queues with the name of the queues as their keys. 
        */
        std::map<string, std::unique_ptr<ROSInConVirtualQueue>> inequality_constraint_virtual_queues_;

        /**
         * @brief Map of equality constraint virtual queues with the name of the queues as their keys. 
        */
        std::map<string, std::unique_ptr<ROSEqConVirtualQueue>> equality_constraint_virtual_queues_;

        /**
         * @brief Map of real queues with the queue size converted in bytes with the name of the queues as their keys. 
        */
        std::map<string, std::unique_ptr<ROSByteConvertedQueue>> real_queues_;

        /**
         * @brief ROS nodle handle to to get the params and to create the services, publisher and subscribers.
        */
        ros::NodeHandle nh_;

        /**
         * @brief Name of the queue server to use a meta data.
        */
        string queue_server_name_;

        /**
         * @brief Publisher that periodically publish and table of all the size of the queues.
        */
        ros::Publisher queue_server_states_pub_;

        /**
         * @brief Service server that provides on demand the size of the queues
        */
        ros::ServiceServer queue_server_states_service_;

        /**
         * @brief Service server that provides on demand the meta information of all queues.
        */
        ros::ServiceServer queue_server_queue_infos_service_;

        /**
         * @brief Service server that updates all the virtual queues on demand.
        */
        ros::ServiceServer queue_server_update_virtual_queues_service;
};

