#pragma once

#include <stdexcept>

using std::invalid_argument;

/**
 * @brief Interface class that wraps a queue with interface to interact with it. 
 * @tparam TQueueType Type of the element in the queue.
 */
template<typename TQueueType>
class IDynamicQueue
{
    public:
        IDynamicQueue(int max_queue_size)
        {
            if (max_queue_size < 0)
            {
                throw invalid_argument("Tried to initiate queue with negative maximum size");
            }
            max_queue_size_ = max_queue_size;
        };

        virtual int getSize(){return internal_queue_.size();};
        virtual int getMemSize()=0;

        virtual int evaluate()=0;
        
        virtual bool update(TQueueType arriving_elements, const int nb_departing_elements)=0;
        
        inline virtual bool isfull(){return internal_queue_.size()==max_queue_size_;};

    protected:
        int max_queue_size_=0;

        TQueueType internal_queue_;

        virtual int arrival_prediction()=0;
        virtual int transmission_prediction()=0;
};