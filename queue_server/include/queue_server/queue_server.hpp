#pragma once

#include <utility>
#include <map>
#include <string>

#include "ros_queue/ros_queue.hpp"
#include "ros_queue/ros_converted_queue.hpp"
#include "ros_queue/ros_virtual_queue.hpp"
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"

#include "ros_queue_msgs/QueueStates.h"
#include "ros_queue_msgs/QueueStatesPrediction.h"

using std::string;

class QueueServer
{
    
    public:
        QueueServer();
        //std::vector<ROSVirtualQueue<InConVirtualQueue, queue_server::QueueStatesPrediction>> virtual_queues_;

        /*QueueServer(std::vector<ROSVirtualQueue<InConVirtualQueue, queue_server::QueueStatesPrediction>>&& virtual_queues):virtual_queues_(std::move(virtual_queues))
        {

        }*/

        
        void addInequalityConstraintVirtualQueue(ROSInConVirtualQueue&& new_queue);
        void addEqualityConstraintVirtualQueue(ROSInConVirtualQueue&& new_queue);

        //std::vector<????> queue_containers
        //ROSVirtualQueue<InConVirtualQueue<>
    private:
        std::map<string, ROSInConVirtualQueue> inequality_constraint_virtual_queues_;
        std::map<string, ROSEqConVirtualQueue> equality_constraint_virtual_queues_;
};

