#pragma once

#include <deque>
#include <mutex>
#include <stdexcept>


#include "ros_queues/lib_queue/I_dynamic_queue.hpp"
#include "ros_queues/lib_queue/element_with_converted_size.hpp"
#include "ros_queues/lib_queue/queue_exception.hpp"

using namespace std;

template<typename TQueueElementType>
class DynamicQueue: public IDynamicQueue<deque<TQueueElementType>>
{
    public:
        DynamicQueue(int max_queue_size): IDynamicQueue<deque<TQueueElementType>>(max_queue_size) {};

        virtual int getSize()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        } 

        virtual int evaluate() override
        {
            const int arrival = arrival_prediction();
            const int departure = transmission_prediction();
            
            if (arrival < 0)
            {
                throw NegativeArrivalPredictionException("");
            }
            if (departure < 0)
            {
                throw NegativeDeparturePredictionException("");
            }

            // Protect access to queue
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            const int current_size = this->internal_queue_.size();

            // Queue dynamic
            int new_size = (current_size > departure) ? current_size - departure + arrival : arrival; 

            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
            }
            
            return new_size;
        }

        bool update(deque<TQueueElementType> arriving_elements, const int departure)
        {
            if(departure < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }
            
            deque<TQueueElementType> queue_to_transmit;
            bool overflowed = false;

            // Inner scope block for the mutex
            {
                // Protect access to queue
                lock_guard<mutex> lock(queue_manipulation_mutex_);

                for(int i =0; i<departure; ++i)
                {
                    if(this->internal_queue_.empty())
                    {
                        break;
                    }
                    queue_to_transmit.push_back(this->internal_queue_.front());
                    this->internal_queue_.pop_front();
                }
                //Receiving data
                while(!arriving_elements.empty())
                {
                    if (this->internal_queue_.size() >= this->max_queue_size_)
                    {
                        overflowed = true;
                        break;
                    }

                    this->internal_queue_.push_back(arriving_elements.front());
                    arriving_elements.pop_front();
                }
            }

            //TODO: Add transmission error handling
            transmit(queue_to_transmit);

            return !overflowed;
        }

        virtual bool transmit(deque<TQueueElementType> &queue_to_transmit) {return 1;};
        
        deque<TQueueElementType> getInternalQueue()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_;
        }

    protected:
        virtual int arrival_prediction() override {return 0;};
        virtual int transmission_prediction() override {return 0;};
        mutex queue_manipulation_mutex_;
};


template<typename TQueueElementType>
class DynamicConvertedQueue: public IDynamicQueue<deque<ElementWithConvertedSize<TQueueElementType>>>
{
    public:
        DynamicConvertedQueue(int max_queue_size,
            void (*conversionFunction)(deque<TQueueElementType>&, deque<ElementWithConvertedSize<TQueueElementType>>&))
            : IDynamicQueue<deque<ElementWithConvertedSize<TQueueElementType>>>(max_queue_size), generateConvertedQueue(conversionFunction) {};

        virtual int getSize()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return converted_queue_size_;
        } 

        virtual int getInternalQueueSize()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        }

        virtual int evaluate() override
        {
            const int arrival = arrival_prediction();
            const int departure = transmission_prediction();
            
            if (arrival < 0)
            {
                throw NegativeArrivalPredictionException("");
            }
            if (departure < 0)
            {
                throw NegativeDeparturePredictionException("");
            }

            // Protect access to queue
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            const int current_size = converted_queue_size_;
            // Queue dynamic
            int new_size = (current_size > departure) ? current_size - departure + arrival : arrival; 

            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
            }
            
            return new_size;
        }

        bool update(deque<ElementWithConvertedSize<TQueueElementType>> arriving_elements, const int departure) override
        {
            if(departure < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            deque<TQueueElementType> queue_to_transmit;
            bool overflowed = false;

            // Inner scope block for the mutex
            {
                // Protect access to queue
                lock_guard<mutex> lock(queue_manipulation_mutex_);

                for(int i =0; i<departure; ++i)
                {
                    if(this->internal_queue_.empty())
                    {
                        break;
                    }
                    
                    converted_queue_size_ -= this->internal_queue_.front().converted_size_;
                    queue_to_transmit.push_back(this->internal_queue_.front().element_);
                    this->internal_queue_.pop_front();
                }
                //Receiving data
                while(!arriving_elements.empty())
                {
                    int size_of_element = arriving_elements.front().converted_size_;
                    
                    if (size_of_element == 0)
                    {
                        throw BadConversionException("Element has a converted size of zero. Likely due to a bad conversion function.");
                    }
                    else if  (size_of_element < 0)
                    {
                        throw BadConversionException("Element has a negative converted size. Likely due to a bad conversion function."); 
                    }

                    if (converted_queue_size_ + size_of_element >= this->max_queue_size_)
                    {
                        overflowed = true;
                        break;
                    }
                    converted_queue_size_ += size_of_element;
                    this->internal_queue_.push_back(arriving_elements.front());
                    arriving_elements.pop_front();
                }
            }

            //TODO: Add transmission error handling
            transmit(queue_to_transmit);

            return !overflowed;
        }

        bool update(deque<TQueueElementType> arriving_elements, const int departure)
        {
            deque<ElementWithConvertedSize<TQueueElementType>> wrapped_arriving_elements;
            const int arriving_elements_size = arriving_elements.size();

            // Verify if given conversion function exist
            if(generateConvertedQueue != nullptr)
            {
                generateConvertedQueue(arriving_elements, wrapped_arriving_elements);
                
                if (arriving_elements_size != wrapped_arriving_elements.size())
                {
                    throw BadConversionException("The size of the arriving elements and the wrapped queue are different. Likely due to a bad conversion function.");
                }
            }
            else
            {
                throw BadConversionException("The conversion function pointer is null.");
            }

            return update(wrapped_arriving_elements, departure);
        }

        virtual bool transmit(deque<TQueueElementType> &queue_to_transmit) {return 1;};
        
        deque<TQueueElementType> getInternalQueue()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            
            deque<TQueueElementType> data_queue;
            for(typename deque<ElementWithConvertedSize<TQueueElementType>>::iterator it = this->internal_queue_.begin(); it != this->internal_queue_.end(); ++it)
            {
                data_queue.push_back(it->element_);
            }

            return data_queue;
        }

    protected:
        int converted_queue_size_ = 0;

        void (*generateConvertedQueue)(deque<TQueueElementType>&, deque<ElementWithConvertedSize<TQueueElementType>>&);
        
        virtual int arrival_prediction() override {return 0;};
        virtual int transmission_prediction() override {return 0;};
        
        mutex queue_manipulation_mutex_;
};