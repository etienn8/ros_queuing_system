#pragma once

#include <deque>
#include <mutex>
#include <stdexcept>

#include "ros_queue/lib_queue/I_dynamic_queue.hpp"
#include "ros_queue/lib_queue/queue_exception.hpp"

using namespace std;

/**
 * @brief Deque of specified type with interfaces to affect its dynamic.
 * @details Wraps a std::deque with interfaces to manipulate and evaluate it. Also contains a mutex to protect the queue from race conditions.  
 * @tparam TQueueElementType Type of the elements in the deque.
 * @tparam TStates Type of the argument of the evaluate function that is passed to the pridictions methods.
 */
template<typename TQueueElementType, typename TStates=void>
class DynamicQueue: public IDynamicQueue<deque<TQueueElementType>, TStates>
{
    public:
        DynamicQueue(int max_queue_size): IDynamicQueue<deque<TQueueElementType>, TStates>(max_queue_size) {};

        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getSize()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        } 

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods IDynamicQueue::arrival_prediction and IDynamicQueue::transmission_prediction.
         * @param states Argument with a type given by a template parameter that is passed to the prediction function so the user can give data to the prediction. 
         * @throw Throws a NegativeArrivalPredictionException if the IDynamicQueue::arrival_prediction predicts an negative number of incoming elements which sould never logicaly happen.
         * @throw Throws a NegativeDeparturePredictionException if the IDynamicQueue::transmission_prediction predicts an negative number of departing elements which sould never logicaly happen.
         * @return Predicted size of the queue after evaluation.
         */
        virtual int evaluate(const TStates& states) override
        {
            const int arrival = arrival_prediction(states);
            const int departure = transmission_prediction(states);
            
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

        /**
         * @brief Updates the queue by adding the arriving_elements and by transmitting the specifiy number of departing elements while respecting the maximum queue size.
         * @details Data are transmitted by using DynamicQueue::transmit.
         * @param arriving_elements Queue of elements to add to the queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        bool update(deque<TQueueElementType> arriving_elements, const int nb_departing_elements) override
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

        /**
         * @brief Get a copy of the internal deque.
         * @return A copy of the internal deque.
         */
        deque<TQueueElementType> getInternalQueue()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_;
        }

    protected:
        /**
         * @brief Method used internaly to transmit data. Override this method to define a specific transmit behavior.  
         * @param queue_to_transmit Queue of elements to transmit.
         * @return Returns if the transmission succeeded or not.
         */
        virtual bool transmit(deque<TQueueElementType> &queue_to_transmit) {return 1;};

        /**
         * @brief Method used in the evaluation process to predict what will be the arrival. Override this method to define a specific arrival prediction behavior.
         * @param states Argument passed by the evaluation process and has a type decided by the template. Allows derived class to use data(states) passed by the application level.
         * @return Returns the size of the estimated arrival queue.
         */
        virtual int arrival_prediction(const TStates& states) override {return 0;};

        /**
         * @brief Method used in the evaluation process to predict how many elements is predicted to depart. Override this method to define a specific transmission prediction behavior.
         * @param states Argument passed by the evaluation process and has a type decided by the template. Allows derived class to use data(states) passed by the application level.
         * @return Returns the size of the queue that could be transmitted.
         */
        virtual int transmission_prediction(const TStates& states) override {return 0;};

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        mutex queue_manipulation_mutex_;
};

/**
 * @brief Deque of specified type with interfaces to affect its dynamic.
 * @details Wraps a std::deque with interfaces to manipulate and evaluate it. Also contains a mutex to protect the queue from race conditions.
 * @tparam TQueueElementType Type of the elements in the deque.
 */
template<typename TQueueElementType>
class DynamicQueue<TQueueElementType, void>: public IDynamicQueue<deque<TQueueElementType>, void>
{
    public:
        DynamicQueue(int max_queue_size): IDynamicQueue<deque<TQueueElementType>>(max_queue_size) {};

        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getSize()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        } 

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods IDynamicQueue::arrival_prediction and IDynamicQueue::transmission_prediction.
         * @throw Throws a NegativeArrivalPredictionException if the IDynamicQueue::arrival_prediction predicts an negative number of incoming elements which sould never logicaly happen.
         * @throw Throws a NegativeDeparturePredictionException if the IDynamicQueue::transmission_prediction predicts an negative number of departing elements which sould never logicaly happen.
         * @return Predicted size of the queue after evaluation.
         */
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

        /**
         * @brief Updates the queue by adding the arriving_elements and by transmitting the specifiy number of departing elements while respecting the maximum queue size.
         * @details Data are transmitted by using DynamicQueue::transmit.
         * @param arriving_elements Queue of elements to add to the queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        bool update(deque<TQueueElementType> arriving_elements, const int nb_departing_elements) override
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

        /**
         * @brief Get a copy of the internal deque.
         * @return A copy of the internal deque.
         */
        deque<TQueueElementType> getInternalQueue()
        {
            lock_guard<mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_;
        }

    protected:
        /**
         * @brief Method used internaly to transmit data. Override this method to define a specific transmit behavior.  
         * @param queue_to_transmit Queue of elements to transmit.
         * @return Returns if the transmission succeeded or not.
         */
        virtual bool transmit(deque<TQueueElementType> &queue_to_transmit) {return 1;};

        /**
         * @brief Method used in the evaluation process to predict what will be the arrival. Override this method to define a specific arrival prediction behavior.
         * @return Returns the size of the estimated arrival queue.
         */
        virtual int arrival_prediction() override {return 0;};

        /**
         * @brief Method used in the evaluation process to predict how many elements is predicted to depart. Override this method to define a specific transmission prediction behavior.
         * @return Returns the size of the queue that could be transmitted.
         */
        virtual int transmission_prediction() override {return 0;};

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        mutex queue_manipulation_mutex_;
};