#pragma once

#include <mutex>

#include "ros_queues/lib_queue/I_dynamic_queue.hpp"
#include "ros_queues/lib_queue/virtual_queue.hpp"

class InConVirtualQueue: public IDynamicQueue<VirtualQueue>
{
    public:
        InConVirtualQueue(unsigned int max_queue_size): IDynamicQueue(max_queue_size) {};
        virtual int getSize() override;
        virtual int getMemSize() override;
        virtual int evaluate() override;
        
        virtual bool update(VirtualQueue arriving_elements, const unsigned int departure) override;
        bool udpate(const int arrival, const unsigned int departure);
    
    protected:
        virtual int arrival_prediction() override;
        virtual int transmission_prediction() override;
        std::mutex queue_manipulation_mutex_;
};

class EqConVirtualQueue: public IDynamicQueue<NVirtualQueue>
{
    public:
        EqConVirtualQueue(unsigned int max_queue_size): IDynamicQueue(max_queue_size) {};
        virtual int getSize() override;
        virtual int getMemSize() override;
        virtual int evaluate() override;
        
        virtual bool update(NVirtualQueue arriving_elements, const unsigned int departure) override;
        bool udpate(const int arrival, const unsigned int departure);
    
    protected:
        virtual int arrival_prediction() override;
        virtual int transmission_prediction() override;
        std::mutex queue_manipulation_mutex_;
};