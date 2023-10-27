#pragma once

#include <deque>
#include <mutex>
#include <stdexcept>

#include "ros_queue/lib_queue/I_dynamic_queue.hpp"
#include "ros_queue/lib_queue/element_with_converted_size.hpp"
#include "ros_queue/lib_queue/queue_exception.hpp"

/**
 * @brief Deque of a specified type with interfaces to affect its dynamic where the size of the queues is the sum of the converted size (computed from a user-defined function) of each of its element.
 * @details Wraps a std::deque with interfaces to manipulate and evaluate it. 
 * Also contains a mutex to protect the queue from race conditions. The conversion is added to store elements of one type but to give the flexibility to define how big the element is or to put the queue in a format closer to the units used by the transmission process.  
 * @tparam TQueueElementType Type of the elements in the deque.
 * @tparam TStates Type of the argument of the evaluate function that is passed to the pridictions methods.
 */
template<typename TQueueElementType, typename TStates=void>
class DynamicConvertedQueue: public IDynamicQueue<deque<ElementWithConvertedSize<TQueueElementType>>, TStates>
{
    public:
        DynamicConvertedQueue(int max_queue_size,
            void (*conversionFunction)(deque<TQueueElementType>&, deque<ElementWithConvertedSize<TQueueElementType>>&))
            : IDynamicQueue<deque<ElementWithConvertedSize<TQueueElementType>>, TStates>(max_queue_size), generateConvertedQueue(conversionFunction) {};

        /**
         * @brief Get the size of the converted size of the queue.
         * @return Return the size converted size of the queue.
         */
        virtual int getSize()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return converted_queue_size_;
        } 

        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getInternalQueueSize()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        }

        /**
         * @brief Evaluate the size of the queue based on the converted size of the queue and predicted arrival and departure rate (in internal queue size instead of converted size).
         * @details It evaluates the size of the queue based on it's actual converted size and predicted arrival and departure rate. Those prediction are evaluated based on the methods IDynamicQueue::arrival_prediction and IDynamicQueue::transmission_prediction.
         * @param states Argument with a type given by a template parameter that is passed to the prediction function so the user can give data to the prediction. 
         * @throw Throws a NegativeArrivalPredictionException if the IDynamicQueue::arrival_prediction predicts an negative number of incoming elements which sould never logicaly happen.
         * @throw Throws a NegativeDeparturePredictionException if the IDynamicQueue::transmission_prediction predicts an negative number of departing elements which sould never logicaly happen.
         * @return Predicted converted size of the queue after evaluation.
         */
        virtual int evaluate(const TStates& states) override
        {
            const int converted_arrival = arrival_prediction(states);
            const int departure = transmission_prediction(states);
            
            if (converted_arrival < 0)
            {
                throw NegativeArrivalPredictionException("");
            }
            if (departure < 0)
            {
                throw NegativeDeparturePredictionException("");
            }

            // Protect access to queue
            lock_guard<mutex> lock(queue_manipulation_mutex_);

            int index = 0;
            int converted_departure = 0;
            for(typename deque<ElementWithConvertedSize<TQueueElementType>>::iterator it = this->internal_queue_.begin(); it != this->internal_queue_.end(); ++it)
            {
                if (index == departure)
                {
                    break;
                }

                converted_departure += it->converted_size_;

                ++index;
            }

            // Queue dynamic
            int new_size = (converted_queue_size_ > converted_departure) ? converted_queue_size_ - converted_departure + converted_arrival : converted_arrival; 

            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
            }
            
            return new_size;
        }

        /**
         * @brief Updates the queue by adding the wrapped arriving_elements and by transmitting the specifiy number of departing elements while respecting the maximum queue size.
         * @details Data are transmitted by using DynamicQueue::transmit. The converted size of the queue is increasing whenever an element is added by its converted cost integrated in the wrapped element object. 
         * @param arriving_elements Queue of wrapped object of an element and its converted size to add to the internal queue.
         * @param nb_departing_elements Number of elements (not in converted size but in internal_queue size) to remove from the internal queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @throw Throws an BadConversionException if the converted size of an element is 0 or negative.  
         * @return Boolean of it the queue overflowed while adding elements.
         */
        bool update(deque<ElementWithConvertedSize<TQueueElementType>> arriving_elements, const int nb_departing_elements) override
        {
            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            deque<TQueueElementType> queue_to_transmit;
            bool overflowed = false;

            // Inner scope block for the mutex
            {
                // Protect access to queue
                lock_guard<mutex> lock(queue_manipulation_mutex_);

                for(int i =0; i<nb_departing_elements; ++i)
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


        /**
         * @brief Wraps incoming elements in a queue with their converted size based on a user-defined function and then updates the queue.
         * @details The incoming elements are stored with a converted cost computed from the user-defined function DynamicConvertedQueue::generateConvertedQueue. Once all the elements are wrapped, the queue is updated by calling DynamicConvertedQueue::update(deque<ElementWithConvertedSize<TQueueElementType>>, const int).
         * @param arriving_elements Queue of object to add to the internal queue.
         * @param nb_departing_elements Number of elements (not in converted size but in internal_queue size) to remove from the internal queue.
         * @throw Throws and invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @throw Throws an BadConversionException if the converted size of an element is 0 or negative, the conversion function doesn't produce a queue of equal size as the queue of incoming data, or the conversion function points toward a null function. 
         * @return Boolean of it the queue overflowed while adding elements.
         */
        bool update(deque<TQueueElementType> arriving_elements, const int nb_departing_elements)
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

            return update(wrapped_arriving_elements, nb_departing_elements);
        }

        /**
         * @brief Method used internaly to transmit data. Override this method to define a specific transmit behavior.  
         * @param queue_to_transmit Queue of elements to transmit.
         * @return Returns if the transmission succeeded or not.
         */
        virtual bool transmit(deque<TQueueElementType> &queue_to_transmit) {return 1;};
        
        /**
         * @brief Get a copy of the internal deque of elements without being wrapped.
         * @return A copy of the internal deque.
         */
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
        /**
         * @brief Represents the sum of the converted size of each element in the internal queue. 
        */
        int converted_queue_size_ = 0;

        /**
         * @brief User-defined function given by reference in the constructor that converts a queue in a another queue where each element is stored with a specific cost choseen by the user. See the details to see implementation tips.
         * @details To work properly and prevent BadConversionException during runtime, follow those requirements: Do not give a pointer to a function that as a lifetime shorter than this queue object, the wrapped queue should be the same size as the input queue size and elements should have non-zero positive converted size.
         * @param deque<TQueueElementType>& Reference of a deque of elements that needs to be converted
         * @param deque<ElementWithConvertedSize<TQueueElementType>>& Reference that serves as an output of the wrapped queue with cost affiliated to each elements. 
         * @throw Might not throw any exception directly, but DynamicConvertedQueue::update() might throw BadConversionException during runtime because of a bad behavior of this user-defined function.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        void (*generateConvertedQueue)(deque<TQueueElementType>&, deque<ElementWithConvertedSize<TQueueElementType>>&);
        
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. Override this method to define a specific arrival prediction behavior.
         * @param states Argument passed by the evaluation process and has a type decided by the template. Allows derived class to use data(states) passed by the application level.
         * @return Returns the converted size of the estimated arrival queue.
         */
        virtual int arrival_prediction(const TStates& states) override {return 0;};

        /**
         * @brief Method used in the evaluation process to predict what will be the transmission size. Override this method to define a specific transmission prediction behavior.
         * @param states Argument passed by the evaluation process and has a type decided by the template. Allows derived class to use data(states) passed by the application level.
         * @return Returns the size of the queue that could be transmitted in internal queue size (not converted).
         */
        virtual int transmission_prediction(const TStates& states) override {return 0;};

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        mutex queue_manipulation_mutex_;
};

/**
 * @brief Deque of a specified type with interfaces to affect its dynamic where the size of the queues is the sum of the converted size (computed from a user-defined function) of each of its element.
 * @details Wraps a std::deque with interfaces to manipulate and evaluate it. 
 * Also contains a mutex to protect the queue from race conditions. The conversion is added to store elements of one type but to give the flexibility to define how big the element is or to put the queue in a format closer to the units used by the transmission process.  
 * @tparam TQueueElementType Type of the elements in the deque.
 */
template<typename TQueueElementType>
class DynamicConvertedQueue<TQueueElementType, void>: public IDynamicQueue<deque<ElementWithConvertedSize<TQueueElementType>>>
{
    public:
        DynamicConvertedQueue(int max_queue_size,
            void (*conversionFunction)(deque<TQueueElementType>&, deque<ElementWithConvertedSize<TQueueElementType>>&))
            : IDynamicQueue<deque<ElementWithConvertedSize<TQueueElementType>>>(max_queue_size), generateConvertedQueue(conversionFunction) {};

        /**
         * @brief Get the size of the converted size of the queue.
         * @return Return the size converted size of the queue.
         */
        virtual int getSize()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return converted_queue_size_;
        } 

        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getInternalQueueSize()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        }

        /**
         * @brief Evaluate the size of the queue based on the converted size of the queue and predicted arrival and departure rate (in internal queue size instead of converted size).
         * @details It evaluates the size of the queue based on it's actual converted size and predicted arrival and departure rate. Those prediction are evaluated based on the methods IDynamicQueue::arrival_prediction and IDynamicQueue::transmission_prediction.
         * @throw Throws a NegativeArrivalPredictionException if the IDynamicQueue::arrival_prediction predicts an negative number of incoming elements which sould never logicaly happen.
         * @throw Throws a NegativeDeparturePredictionException if the IDynamicQueue::transmission_prediction predicts an negative number of departing elements which sould never logicaly happen.
         * @return Predicted converted size of the queue after evaluation.
         */
        virtual int evaluate() override
        {
            const int converted_arrival = arrival_prediction();
            const int departure = transmission_prediction();
            
            if (converted_arrival < 0)
            {
                throw NegativeArrivalPredictionException("");
            }
            if (departure < 0)
            {
                throw NegativeDeparturePredictionException("");
            }

            // Protect access to queue
            lock_guard<mutex> lock(queue_manipulation_mutex_);

            int index = 0;
            int converted_departure = 0;
            for(typename deque<ElementWithConvertedSize<TQueueElementType>>::iterator it = this->internal_queue_.begin(); it != this->internal_queue_.end(); ++it)
            {
                if (index == departure)
                {
                    break;
                }

                converted_departure += it->converted_size_;

                ++index;
            }

            // Queue dynamic
            int new_size = (converted_queue_size_ > converted_departure) ? converted_queue_size_ - converted_departure + converted_arrival : converted_arrival; 

            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
            }
            
            return new_size;
        }

        /**
         * @brief Updates the queue by adding the wrapped arriving_elements and by transmitting the specifiy number of departing elements while respecting the maximum queue size.
         * @details Data are transmitted by using DynamicQueue::transmit. The converted size of the queue is increasing whenever an element is added by its converted cost integrated in the wrapped element object. 
         * @param arriving_elements Queue of wrapped object of an element and its converted size to add to the internal queue.
         * @param nb_departing_elements Number of elements (not in converted size but in internal_queue size) to remove from the internal queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @throw Throws an BadConversionException if the converted size of an element is 0 or negative.  
         * @return Boolean of it the queue overflowed while adding elements.
         */
        bool update(deque<ElementWithConvertedSize<TQueueElementType>> arriving_elements, const int nb_departing_elements) override
        {
            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            deque<TQueueElementType> queue_to_transmit;
            bool overflowed = false;

            // Inner scope block for the mutex
            {
                // Protect access to queue
                lock_guard<mutex> lock(queue_manipulation_mutex_);

                for(int i =0; i<nb_departing_elements; ++i)
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


        /**
         * @brief Wraps incoming elements in a queue with their converted size based on a user-defined function and then updates the queue.
         * @details The incoming elements are stored with a converted cost computed from the user-defined function DynamicConvertedQueue::generateConvertedQueue. Once all the elements are wrapped, the queue is updated by calling DynamicConvertedQueue::update(deque<ElementWithConvertedSize<TQueueElementType>>, const int).
         * @param arriving_elements Queue of object to add to the internal queue.
         * @param nb_departing_elements Number of elements (not in converted size but in internal_queue size) to remove from the internal queue.
         * @throw Throws and invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @throw Throws an BadConversionException if the converted size of an element is 0 or negative, the conversion function doesn't produce a queue of equal size as the queue of incoming data, or the conversion function points toward a null function. 
         * @return Boolean of it the queue overflowed while adding elements.
         */
        bool update(deque<TQueueElementType> arriving_elements, const int nb_departing_elements)
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

            return update(wrapped_arriving_elements, nb_departing_elements);
        }

        /**
         * @brief Method used internaly to transmit data. Override this method to define a specific transmit behavior.  
         * @param queue_to_transmit Queue of elements to transmit.
         * @return Returns if the transmission succeeded or not.
         */
        virtual bool transmit(deque<TQueueElementType> &queue_to_transmit) {return 1;};
        
        /**
         * @brief Get a copy of the internal deque of elements without being wrapped.
         * @return A copy of the internal deque.
         */
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
        /**
         * @brief Represents the sum of the converted size of each element in the internal queue. 
        */
        int converted_queue_size_ = 0;

        /**
         * @brief User-defined function given by reference in the constructor that converts a queue in a another queue where each element is stored with a specific cost choseen by the user. See the details to see implementation tips.
         * @details To work properly and prevent BadConversionException during runtime, follow those requirements: Do not give a pointer to a function that as a lifetime shorter than this queue object, the wrapped queue should be the same size as the input queue size and elements should have non-zero positive converted size.
         * @param deque<TQueueElementType>& Reference of a deque of elements that needs to be converted
         * @param deque<ElementWithConvertedSize<TQueueElementType>>& Reference that serves as an output of the wrapped queue with cost affiliated to each elements. 
         * @throw Might not throw any exception directly, but DynamicConvertedQueue::update() might throw BadConversionException during runtime because of a bad behavior of this user-defined function.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        void (*generateConvertedQueue)(deque<TQueueElementType>&, deque<ElementWithConvertedSize<TQueueElementType>>&);
        
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. Override this method to define a specific arrival prediction behavior.
         * @return Returns the converted size of the estimated arrival queue.
         */
        virtual int arrival_prediction() override {return 0;};

        /**
         * @brief Method used in the evaluation process to predict what will be the transmission size. Override this method to define a specific transmission prediction behavior.
         * @return Returns the size of the queue that could be transmitted in internal queue size (not converted).
         */
        virtual int transmission_prediction() override {return 0;};

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        mutex queue_manipulation_mutex_;
};