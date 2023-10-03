#pragma once

#include <queue>
#include "ros_queues/lib_queue/I_dynamic_queue.hpp"

using std::queue;

template<typename TQueueElementType>
class DynamicQueue: public IDynamicQueue<queue<TQueueElementType>>
{
    public:
        DynamicQueue(unsigned int max_queue_size): IDynamicQueue<queue<TQueueElementType>>(max_queue_size) {};

        virtual int getMemSize() override {return this->internal_queue_.size()*sizeof(TQueueElementType);};

        virtual int evaluate() override
        {
            const int arrival = arrival_prediction();
            const int departure = transmission_prediction();
            const int current_size = this->internal_queue_.size();

            // Queue dynamic
            int new_size = (current_size > departure) ? current_size - departure + arrival : arrival; 

            if (current_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
            }
            
            return new_size;
        }
        
        virtual bool update(queue<TQueueElementType> arriving_elements, const unsigned int departure) override
        {
            //Transmit data of queue
            queue<TQueueElementType> queue_to_transmit;
            for(int i =0; i<departure; ++i)
            {
                if(this->internal_queue_.empty())
                {
                    break;
                }
                queue_to_transmit.push(this->internal_queue_.front());
                this->internal_queue_.pop();
            }
            //TODO: Add transmission error handling
            transmit(queue_to_transmit);
            
            bool overflowed = false;
            //Receiving data
            while(!arriving_elements.empty())
            {
                if (this->internal_queue_.size() >= this->max_queue_size_)
                {
                    overflowed = true;
                    break;
                }

                this->internal_queue_.push(arriving_elements.front());
                arriving_elements.pop();
            }
            return !overflowed;
        }

        virtual bool transmit(queue<TQueueElementType> &queue_to_transmit) {return 1;};

    protected:
        virtual int arrival_prediction() override {return 0;};
        virtual int transmission_prediction() override {return 0;};
};


/*template<typename TQueueElementType,typename TMememoryTranslatedType>
class DynamicTranslatedQueue: public IDynamicQueue<queue<TQueueElementType>>
{
    public:
        DynamicTranslatedQueue(unsigned int max_queue_size): IDynamicQueue(max_queue_size) {};

        virtual int getMemSize() override {return sizeof(TQueueElementType)*internal_queue_.size()};
        int getTranslatedSize() const;
        
        
        virtual int evaluate() override;
        
        virtual bool update(queue<TQueueElementType> arriving_elements, const unsigned int departure) override
        {
            //Transmit data of queue
            queue<TMememoryTranslatedType> queue_to_transmit;
            for(int i =0; i<departure; ++i)
            {
                if(internal_queue_.empty())
                {
                    break;
                }
                queue_to_transmit.push(internal_queue_.front());
                internal_queue_.pop();
            }
            //TODO: Add transmission error handling
            transmit(queue_to_transmit);
            
            bool overflowed = false;
            //Receiving data
            while(!arriving_elements.empty())
            {
                if (internal_queue_.size() <= max_queue_size_)
                {
                    overflowed = true;
                    break;
                }
                internal_queue_.push(arriving_elements.front());
                arriving_elements.pop();
            }
            return !overflowed;
        }

        virtual bool transmit(queue<TQueueElementType> &queue_to_transmit);

    protected:
        virtual int arrival_prediction() override;
        virtual int transmission_prediction() override;
};*/
