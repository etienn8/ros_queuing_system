#pragma once

#include <deque>
#include "ros_queues/lib_queue/I_dynamic_queue.hpp"
#include "ros_queues/lib_queue/protected_deque.hpp"
#include "ros_queues/lib_queue/element_with_converted_size.hpp"

using std::deque;

template<typename TQueueElementType>
class DynamicQueue: public IDynamicQueue<ProtectedDeque<TQueueElementType>>
{
    public:
        DynamicQueue(unsigned int max_queue_size): IDynamicQueue<ProtectedDeque<TQueueElementType>>(max_queue_size) {};

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
        
        virtual bool update(ProtectedDeque<TQueueElementType> arriving_elements, const unsigned int departure) override
        {
            //Transmit data of queue
            deque<TQueueElementType> queue_to_transmit;
            for(int i =0; i<departure; ++i)
            {
                if(this->internal_queue_.empty())
                {
                    break;
                }
                queue_to_transmit.push_back(this->internal_queue_.front());
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

        bool update(deque<TQueueElementType> arriving_elements, const unsigned int departure)
        {
            //Transmit data of queue
            deque<TQueueElementType> queue_to_transmit;
            for(int i =0; i<departure; ++i)
            {
                if(this->internal_queue_.empty())
                {
                    break;
                }
                queue_to_transmit.push_back(this->internal_queue_.front());
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
                arriving_elements.pop_front();
            }
            return !overflowed;
        }

        virtual bool transmit(deque<TQueueElementType> &queue_to_transmit) {return 1;};

    protected:
        virtual int arrival_prediction() override {return 0;};
        virtual int transmission_prediction() override {return 0;};
};


/*template<typename TQueueElementType>
class DynamicConvertedQueue: public IDynamicQueue<ProtectedDeque<ElementWithConvertedSize<TQueueElementType>>>
{
    public:
        DynamicConvertedQueue(unsigned int max_queue_size): IDynamicQueue<ElementWithConvertedSize<TQueueElementType>>(max_queue_size) {};

        virtual int getMemSize() override {return this->internal_queue_.size()*sizeof(TQueueElementType);};
        
        int getConvertedQueueSize()
        {
            static_cast<int> 
        }

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
        
        virtual bool update(ProtectedDeque<TQueueElementType> arriving_elements, const unsigned int departure) override
        {
            //Transmit data of queue
            deque<TQueueElementType> queue_to_transmit;
            for(int i =0; i<departure; ++i)
            {
                if(this->internal_queue_.empty())
                {
                    break;
                }
                queue_to_transmit.push_back(this->internal_queue_.front());
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

        bool update(deque<TQueueElementType> arriving_elements, const unsigned int departure)
        {
            //Transmit data of queue
            deque<TQueueElementType> queue_to_transmit;
            for(int i =0; i<departure; ++i)
            {
                if(this->internal_queue_.empty())
                {
                    break;
                }
                queue_to_transmit.push_back(this->internal_queue_.front());
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
                arriving_elements.pop_front();
            }
            return !overflowed;
        }

        virtual bool transmit(deque<TQueueElementType> &queue_to_transmit) {return 1;};

    protected:
        virtual int arrival_prediction() override {return 0;};
        virtual int transmission_prediction() override {return 0;};
};*/