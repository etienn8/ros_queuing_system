#pragma once

#include <utility>
#include <vector>

#include "ros_queue/ros_queue.hpp"
#include "ros_queue/ros_converted_queue.hpp"
#include "ros_queue/ros_virtual_queue.hpp"
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"

#include "queue_server/QueueStates.h"
#include "queue_server/QueueStatesPrediction.h"

/*typedef ROSQueue<ros_queue::queue_transmit_template, ros_queue::ReturnSentValue> ROSIntQueue_g; 

std::map<std::string, const std::type_info> queue_map_type_g = {
    {"observation_queue", ROSIntQueue_g}
};*/

class QueueServer
{
    
    public:
        std::vector<ROSVirtualQueue<InConVirtualQueue, queue_server::QueueStatesPrediction>> virtual_queues_;

        /*QueueServer(std::vector<ROSVirtualQueue<InConVirtualQueue, queue_server::QueueStatesPrediction>>&& virtual_queues):virtual_queues_(std::move(virtual_queues))
        {

        }*/

        
        
        //std::vector<????> queue_containers
        //ROSVirtualQueue<InConVirtualQueue<>
};

