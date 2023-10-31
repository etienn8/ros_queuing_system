#pragma once

#include <mutex>
#include <stdexcept>

#include "ros_queue/lib_queue/I_dynamic_queue.hpp"
#include "ros_queue/lib_queue/virtual_queue.hpp"
#include "ros_queue/lib_queue/queue_exception.hpp"

using std::invalid_argument;

/**
 * @brief Virtual queue with interfaces to affect its dynamic. Implemented like virtual queues used with inequality constraint where their size can't go below 0.
 * @tparam TStates Type of the argument of the evaluate function that is passed to the pridictions methods.
 */
template <typename TStates=void>
class InConVirtualQueue: public IDynamicQueue<VirtualQueue, TStates>
{
    public:
        InConVirtualQueue(int max_queue_size): IDynamicQueue<VirtualQueue, TStates>(max_queue_size) {};
        
        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getSize() override
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        };

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods InConVirtualQueue::arrival_prediction and InConVirtualQueue::transmission_prediction.
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

            //Protect access to queue
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

            const int current_size = this->internal_queue_.size();

            // Queue dynamic
            int new_size = (current_size > departure) ? current_size - departure + arrival : arrival; 

            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
            }
            
            return new_size;
        };
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param arriving_elements Virtual queue to append to the internal virtual queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(VirtualQueue arriving_elements, const int nb_departing_elements) override
        {
            const int arrival = arriving_elements.size();
            
            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            // Protect access to queue
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

            const int current_size = this->internal_queue_.size();

            // Queue dynamic
            int new_size = (current_size > nb_departing_elements) ? current_size - nb_departing_elements + arrival : arrival; 

            bool overflowed = false;
            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
                overflowed = true;
            }

            this->internal_queue_.setSize(new_size);

            return !overflowed;
        };
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param nb_arriving_elements Number of elements to add to queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(const int nb_arriving_elements, const int nb_departing_elements)
        {
            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            if(nb_arriving_elements < 0)
            {
                throw invalid_argument("Tried to add a negative number of elements from the queue.");
            }

            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

            const int current_size = this->internal_queue_.size();

            // Queue dynamic
            int new_size = (current_size > nb_departing_elements) ? current_size - nb_departing_elements + nb_arriving_elements : nb_arriving_elements; 

            bool overflowed = false;
            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
                overflowed = true;
            }

            this->internal_queue_.setSize(new_size);
            return !overflowed;
        };

    protected:
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. Override this method to define a specific arrival prediction behavior.
         * @param states Argument passed by the evaluation process and has a type decided by the template. Allows derived class to use data(states) passed by the application level.
         * @return Returns the number of elements that could be added to the queue.
         */
        virtual int arrival_prediction(const TStates& states) override {return 0;};

        /**
         * @brief Method used in the evaluation process to predict what will be the departure size. Override this method to define a specific transmission prediction behavior.
         * @param states Argument passed by the evaluation process and has a type decided by the template. Allows derived class to use data(states) passed by the application level.
         * @return Returns the number of elements that could be removed from the queue.
         */
        virtual int transmission_prediction(const TStates& states) override {return 0;};

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        std::mutex queue_manipulation_mutex_;
};

/**
 * @brief Virtual queue with interfaces to affect its dynamic. Implemented like virtual queues used with inequality constraint where their size can't go below 0. 
 */
template <>
class InConVirtualQueue<void>: public IDynamicQueue<VirtualQueue>
{
    public:
        InConVirtualQueue(int max_queue_size): IDynamicQueue<VirtualQueue>(max_queue_size) {};
        
        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getSize() override
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        };

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods InConVirtualQueue::arrival_prediction and InConVirtualQueue::transmission_prediction.
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

            //Protect access to queue
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

            const int current_size = this->internal_queue_.size();

            // Queue dynamic
            int new_size = (current_size > departure) ? current_size - departure + arrival : arrival; 

            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
            }
            
            return new_size;
        };
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param arriving_elements Virtual queue to append to the internal virtual queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(VirtualQueue arriving_elements, const int nb_departing_elements) override
        {
            const int arrival = arriving_elements.size();
            
            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            // Protect access to queue
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

            const int current_size = this->internal_queue_.size();

            // Queue dynamic
            int new_size = (current_size > nb_departing_elements) ? current_size - nb_departing_elements + arrival : arrival; 

            bool overflowed = false;
            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
                overflowed = true;
            }

            this->internal_queue_.setSize(new_size);

            return !overflowed;
        };
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param nb_arriving_elements Number of elements to add to queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(const int nb_arriving_elements, const int nb_departing_elements)
        {
            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            if(nb_arriving_elements < 0)
            {
                throw invalid_argument("Tried to add a negative number of elements from the queue.");
            }

            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

            const int current_size = this->internal_queue_.size();

            // Queue dynamic
            int new_size = (current_size > nb_departing_elements) ? current_size - nb_departing_elements + nb_arriving_elements : nb_arriving_elements; 

            bool overflowed = false;
            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
                overflowed = true;
            }

            this->internal_queue_.setSize(new_size);
            return !overflowed;
        };

    protected:
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. Override this method to define a specific arrival prediction behavior.
         * @return Returns the number of elements that could be added to the queue.
         */
        virtual int arrival_prediction() override {return 0;};

        /**
         * @brief Method used in the evaluation process to predict what will be the departure size. Override this method to define a specific transmission prediction behavior.
         * @return Returns the number of elements that could be removed from the queue.
         */
        virtual int transmission_prediction() override {return 0;};

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        std::mutex queue_manipulation_mutex_;
};

/**
 * @brief Virtual queue with interfaces to affect its dynamic. Implemented like virtual queues used with equality constraint where their size can go below 0. 
 * @tparam TStates Type of the argument of the evaluate function that is passed to the pridictions methods.
 */
template <typename TState=void>
class EqConVirtualQueue: public IDynamicQueue<NVirtualQueue, TState>
{
    public:
        EqConVirtualQueue(int max_queue_size): IDynamicQueue<NVirtualQueue, TState>(max_queue_size) {};
        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getSize() override
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        };

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods InConVirtualQueue::arrival_prediction and InConVirtualQueue::transmission_prediction.
         * @param states Argument with a type given by a template parameter that is passed to the prediction function so the user can give data to the prediction. 
         * @throw Throws a NegativeArrivalPredictionException if the IDynamicQueue::arrival_prediction predicts an negative number of incoming elements which sould never logicaly happen.
         * @throw Throws a NegativeDeparturePredictionException if the IDynamicQueue::transmission_prediction predicts an negative number of departing elements which sould never logicaly happen.
         * @return Predicted size of the queue after evaluation.
         */
        virtual int evaluate(const TState& states) override
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

            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            const int current_size = this->internal_queue_.size();

            // Dynamics of a virtual queue that constrain a  time average value at zero
            int new_size = current_size - departure + arrival; 

            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
            }
            else if (new_size < -this->max_queue_size_)
            {
                new_size = -this->max_queue_size_;
            }
            
            return new_size;
        };

        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param arriving_elements Virtual queue to append to the internal virtual queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(NVirtualQueue arriving_elements, const int nb_departing_elements) override
        {
            const int arrival =  arriving_elements.size();

            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            const int current_size = this->internal_queue_.size();

            // Dynamics of a virtual queue that constrain a  time average value at zero
            int new_size = current_size - nb_departing_elements + arrival; 

            bool overflowed = false;
            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
                overflowed = true;
            }
            else if (new_size < -this->max_queue_size_)
            {
                new_size = -this->max_queue_size_;
                overflowed = true;
            }

            this->internal_queue_.setSize(new_size);
            return !overflowed;
        };

        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param nb_arriving_elements Number of elements to add to queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(const int nb_arriving_elements, const int nb_departing_elements)
        {
            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            if(nb_arriving_elements < 0)
            {
                throw invalid_argument("Tried to add a negative number of elements from the queue.");
            }

            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            const int current_size = this->internal_queue_.size();

            // Dynamics of a virtual queue that constrain a  time average value at zero
            int new_size = current_size - nb_departing_elements + nb_arriving_elements; 

            bool overflowed = false;
            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
                overflowed = true;
            }
            else if (new_size < -this->max_queue_size_)
            {
                new_size = -this->max_queue_size_;
                overflowed = true;
            }

            this->internal_queue_.setSize(new_size);
            return !overflowed;
        };

    protected:
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. Override this method to define a specific arrival prediction behavior.
         * @return Returns the number of elements that could be added to the queue.
         */
        virtual int arrival_prediction(const TState& states) override {return 0;};

        /**
         * @brief Method used in the evaluation process to predict what will be the departure size. Override this method to define a specific transmission prediction behavior.
         * @return Returns the number of elements that could be removed from the queue.
         */
        virtual int transmission_prediction(const TState& states) override {return 0;};

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        std::mutex queue_manipulation_mutex_;
};

template<>
class EqConVirtualQueue<void>: public IDynamicQueue<NVirtualQueue>
{
    public:
        EqConVirtualQueue(int max_queue_size): IDynamicQueue<NVirtualQueue>(max_queue_size) {};
        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getSize() override
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        };

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods InConVirtualQueue::arrival_prediction and InConVirtualQueue::transmission_prediction.
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

            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            const int current_size = this->internal_queue_.size();

            // Dynamics of a virtual queue that constrain a  time average value at zero
            int new_size = current_size - departure + arrival; 

            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
            }
            else if (new_size < -this->max_queue_size_)
            {
                new_size = -this->max_queue_size_;
            }
            
            return new_size;
        };

        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param arriving_elements Virtual queue to append to the internal virtual queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(NVirtualQueue arriving_elements, const int nb_departing_elements) override
        {
            const int arrival =  arriving_elements.size();

            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            const int current_size = this->internal_queue_.size();

            // Dynamics of a virtual queue that constrain a  time average value at zero
            int new_size = current_size - nb_departing_elements + arrival; 

            bool overflowed = false;
            if (new_size > max_queue_size_)
            {
                new_size = max_queue_size_;
                overflowed = true;
            }
            else if (new_size < -max_queue_size_)
            {
                new_size = -max_queue_size_;
                overflowed = true;
            }

            this->internal_queue_.setSize(new_size);
            return !overflowed;
        };

        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param nb_arriving_elements Number of elements to add to queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(const int nb_arriving_elements, const int nb_departing_elements)
        {
            if(nb_departing_elements < 0)
            {
                throw invalid_argument("Tried to remove a negative number of elements from the queue.");
            }

            if(nb_arriving_elements < 0)
            {
                throw invalid_argument("Tried to add a negative number of elements from the queue.");
            }

            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            const int current_size = this->internal_queue_.size();

            // Dynamics of a virtual queue that constrain a  time average value at zero
            int new_size = current_size - nb_departing_elements + nb_arriving_elements; 

            bool overflowed = false;
            if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
                overflowed = true;
            }
            else if (new_size < -this->max_queue_size_)
            {
                new_size = -this->max_queue_size_;
                overflowed = true;
            }

            this->internal_queue_.setSize(new_size);
            return !overflowed;
        };

    protected:
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. Override this method to define a specific arrival prediction behavior.
         * @param states Argument passed by the evaluation process and has a type decided by the template. Allows derived class to use data(states) passed by the application level.
         * @return Returns the number of elements that could be added to the queue.
         */
        virtual int arrival_prediction() override {return 0;};

        /**
         * @brief Method used in the evaluation process to predict what will be the departure size. Override this method to define a specific transmission prediction behavior.
         * @param states Argument passed by the evaluation process and has a type decided by the template. Allows derived class to use data(states) passed by the application level.
         * @return Returns the number of elements that could be removed from the queue.
         */
        virtual int transmission_prediction() override {return 0;};

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        std::mutex queue_manipulation_mutex_;
};