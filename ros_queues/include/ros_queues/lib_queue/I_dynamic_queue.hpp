#pragma once

template<typename TQueueType>
class IDynamicQueue
{
    public:
        IDynamicQueue(unsigned int max_queue_size):max_queue_size_(max_queue_size){};

        virtual int getSize(){return internal_queue_.size();};
        virtual int getMemSize()=0;

        virtual int evaluate()=0;
        
        virtual bool update(TQueueType arriving_elements, const unsigned int nb_departing_elements)=0;
        
        inline virtual bool isfull(){return internal_queue_.size()==max_queue_size_;};

    protected:
        int max_queue_size_;

        TQueueType internal_queue_;

        virtual int arrival_prediction()=0;
        virtual int transmission_prediction()=0;
};