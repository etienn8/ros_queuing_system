#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "ros_queue/ros_byte_converted_queue.hpp"
#include "ros_queue/ros_virtual_queue.hpp"
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"

#include "ros_queue_msgs/QueueStates.h"
#include "ros_queue_msgs/QueueStatesPrediction.h"

using std::string;

class QueueServer
{
    public:
        QueueServer(ros::NodeHandle& nh);
        
        void addInequalityConstraintVirtualQueue(std::unique_ptr<ROSInConVirtualQueue>&& new_queue);
        void addEqualityConstraintVirtualQueue(std::unique_ptr<ROSEqConVirtualQueue>&& new_queue);
        void addRealQueue(std::unique_ptr<ROSByteConvertedQueue>&& new_queue);

    private:
        std::map<string, std::unique_ptr<ROSInConVirtualQueue>> inequality_constraint_virtual_queues_;
        std::map<string, std::unique_ptr<ROSEqConVirtualQueue>> equality_constraint_virtual_queues_;
        std::map<string, std::unique_ptr<ROSByteConvertedQueue>> real_queues_;

        ros::NodeHandle nh_;
};

