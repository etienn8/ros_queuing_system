#include "queue_server/queue_server.hpp"

#include <string>
#include <utility>
#include <map>

#include "ros_queue/ros_virtual_queue.hpp"
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"

using std::string;

QueueServer::QueueServer(ros::NodeHandle& nh): nh_(nh)
{

}

void QueueServer::addInequalityConstraintVirtualQueue(std::unique_ptr<ROSInConVirtualQueue>&& new_queue)
{
    // Extract the queue name
    std::string queue_name = std::move(new_queue->info_.queue_name);
    
    // Emplace the new queue into the map
    inequality_constraint_virtual_queues_.emplace(queue_name, std::move(new_queue));
}

void QueueServer::addEqualityConstraintVirtualQueue(std::unique_ptr<ROSEqConVirtualQueue>&& new_queue)
{
    // Extract the queue name
    std::string queue_name = std::move(new_queue->info_.queue_name);
    
    // Emplace the new queue into the map
    equality_constraint_virtual_queues_.emplace(queue_name, std::move(new_queue));
}

void QueueServer::addRealQueue(std::unique_ptr<ROSByteConvertedQueue>&& new_queue)
{
    // Extract the queue name
    std::string queue_name = std::move(new_queue->info_.queue_name);
    
    // Emplace the new queue into the map
    real_queues_.emplace(queue_name, std::move(new_queue));
}