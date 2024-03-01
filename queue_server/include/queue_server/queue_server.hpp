#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "ros/ros.h"

#include "ros_queue/ros_byte_converted_queue.hpp"
#include "ros_queue/ros_virtual_queue.hpp"
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"

#include "ros_queue_msgs/QueueState.h"
#include "ros_queue_msgs/QueueServerStateFetch.h"
#include "ros_queue_msgs/VirtualQueueChangesList.h"

#include "std_srvs/Empty.h"


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
         * @param spin_rate Rate (event per second) at which the real queues will be checked for transmission 
         * and the size of all queues will be published.
        */
        QueueServer(ros::NodeHandle& nh, float spin_rate);
        
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

        /**
         * @brief Executes all the periodic tasks of the server like publishing the server state.
        */
        void serverSpin(const ros::TimerEvent& event);

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
            float max_queue_size_ = 0.0f;

            string arrival_evaluation_service_name_ = "";
            string departure_evaluation_service_name_ = "";

            string arrival_topic_name_ = "";
            string tranmission_topic_name_ = "";
            string transmission_evaluation_service_name_ = "";
        };
        
        /**
         * @brief Checks if the required parameters for a given queue type are defined,
         *  logs if a parameter is missing and creates a queue in its respective map if they are all defined.
         * @param queue_param_struct Structure that contains all the possible queue configs.
        */
        void checkAndCreateQueue(QueueParamStruct& queue_param_struct);

        /**
         * @brief Loads the parameters of the queues from the ROS param server and call methods to create the queues.
        */
        void loadQueueParametersAndCreateQueues();

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
         * @brief Parameter that indicate if queues should compute additionnal statistics via a topic.
         * Only useful at configuration time.
        */
        bool should_queues_compute_stats_ = false;

        /****************************************************************************
         *  ROS interfaces.
        */

        /**
         * @brief Publisher that periodically publishes a list of all the queues and stats about their
         * time average metrics.
        */
        ros::Publisher queue_server_stats_pub_;

        /**
         * @brief Publisher that periodically publishes a table of all the size of the queues.
        */
        ros::Publisher queue_server_states_pub_;

        /**
         * @brief Adds all the name of the queues and their size to a server state ROS message.
         * @tparam TQueueType Type of the queue that has a getSize() method and a member info_.queue_name.
         * @param internal_queues Map of queues from which to iterate and add the queue's name and size to the ros message.
         * @param server_state_msg reference to the message to which the queue state will be append. Serves as an ouptut. 
        */
        template <typename TQueueType>
        void appendQueueSizesToMsg(TQueueType& internal_queues, ros_queue_msgs::QueueServerState& server_state_msg)
        {
            for(auto it = internal_queues.begin(); it != internal_queues.end(); ++it)
            {
                ros_queue_msgs::QueueState new_queue_size_msg;
                new_queue_size_msg.queue_name = it->second->info_.queue_name;
                new_queue_size_msg.current_size = it->second->getSize();

                server_state_msg.queue_sizes.push_back(std::move(new_queue_size_msg));
            }
        }

        /**
         * @brief Publishes a ROS message with the queue_server_states_pub_ to indicate the server state and the size of the queues.
        */
        void publishServerStates();

        /**
         * @brief Publishes a ROS that contrains time averages statistics about the queues if the 
         * stats publishing flag is set  
        */
        void publishServerStats();

        /**
         * @brief For each real queues, verify how much data could be sent and the queue will transmit up to that quanity if possible.
        */
        void transmitRealQueues();

        /**
         * @brief Timer that will call the server spin at rate given in the constructor.
        */
        ros::Timer spin_timer_;

        /**
         * @brief Service server that provides on demand the size of the queues
        */
        ros::ServiceServer queue_server_states_service_;

        /**
         * @brief Callback function that returns the queue server states including the size of the queues.
        */
        bool serverStateCallback(ros_queue_msgs::QueueServerStateFetch::Request& req, ros_queue_msgs::QueueServerStateFetch::Response& res);

        /**
         * @brief Service server that provides on demand the meta information of all queues.
        */
        ros::ServiceServer queue_server_queue_infos_service_;

        /**
         * @brief Service server that updates all the virtual queues on demand.
        */
        ros::ServiceServer queue_server_update_virtual_queues_service_;

        /**
         * @brief Callback of the virtual_queue_manual_changes_ that updates the virtual queues based 
         * on manual changes specified in the received message. If a queue name doesn't match an existing queue
         * the entry will be ignored.
         * @param msg Received msg that contains a list of virtual queue names and changes to be applied.
        */
        void virtualQueuesManualChangesCallback(const ros_queue_msgs::VirtualQueueChangesList::ConstPtr& msg);

        /**
         * @brief ROS topic subscriber that can change the virtual queues based on specify changes.
        */
        ros::Subscriber virtual_queue_manual_changes_;

        /**
         * @brief Callback function of an empty service that updates all the virtual queues. 
         * Since the virtual queues call services, the called serviced should be short.
        */
        bool queueUpdateCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res);
};

