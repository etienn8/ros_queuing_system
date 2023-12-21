#pragma once

#include <mutex>
#include <stdexcept>

#include "float_compare.hpp"

#include "I_dynamic_virtual_queue.hpp"
#include "ros_queue/lib_queue/virtual_queue.hpp"
#include "ros_queue/lib_queue/queue_exception.hpp"


using std::invalid_argument;

/**
 * @brief Virtual queue with interfaces to affect its dynamic. Implemented like virtual queues used with inequality constraint where its size can't go below 0.
 */
class InConVirtualQueue: public IDynamicVirtualQueue
{
    public:
        InConVirtualQueue(float max_queue_size): IDynamicVirtualQueue(max_queue_size) {};
        
        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual float getSize() override
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        };

        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param change Change in the queue that wants to be added to the queue
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(const float change) override
        {
            // Protect access to queue
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

            const float current_size = this->internal_queue_.size();

            // @TODO: Add a real citation 
            // Queue dynamic based on p.56 from Stochastic Network Optimization with Application to Communication and Queueing Systems from Michael J. Neely
            float new_size = current_size + change;

            bool overflowed = false;
            if (new_size < 0.0f)
            {
                new_size = 0.0f;
            } 
            else if (new_size > this->max_queue_size_)
            {
                new_size = this->max_queue_size_;
                overflowed = true;
            }

            this->internal_queue_.setSize(new_size);

            return !overflowed;
        };

        virtual bool isfull() override
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return float_compare(this->internal_queue_.size(), this->max_queue_size_);
        }

    protected:
        /**
         * @brief Mutex to protect access to the internal queue.
         */
        std::mutex queue_manipulation_mutex_;

        /**
         * @brief Queue object used internaly.
         **/
        VirtualQueue internal_queue_;
};

/**
 * @brief Virtual queue with interfaces to affect its dynamic. Implemented like virtual queues used with equality constraint where its size can go below 0.
 */
class EqConVirtualQueue: public IDynamicVirtualQueue
{
    public:
        EqConVirtualQueue(float max_queue_size): IDynamicVirtualQueue(max_queue_size) {};
        
        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual float getSize() override
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return this->internal_queue_.size();
        };

        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param change Change in the queue that wants to be added to the queue
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(const float change) override
        {
            // Protect access to queue
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

            const float current_size = this->internal_queue_.size();

            // @TODO: Add a real citation 
            // Queue dynamic based on p.56 from Stochastic Network Optimization with Application to Communication and Queueing Systems from Michael J. Neely
            float new_size = current_size + change;

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

        virtual bool isfull() override
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return float_compare(this->internal_queue_.size(), this->max_queue_size_) || float_compare(this->internal_queue_.size(), -this->max_queue_size_);
        }

    protected:
        /**
         * @brief Mutex to protect access to the internal queue.
         */
        std::mutex queue_manipulation_mutex_;

        /**
         * @brief Queue object used internaly.
         **/
        NVirtualQueue internal_queue_;
};