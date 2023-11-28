#pragma once

#include <stdexcept>

#include "virtual_queue.hpp"

/**
 * @brief Interface class of interactions that could be made to a virtual queue. 
 */
class IDynamicVirtualQueue
{
    public:
        /**
         * @brief Constructor where the max_queue_size is initialized and checked. 
         * @param max_queue_size Maximum size that the queue can reach.
         * @throw Throws an invalid_argument if the specified queue_size is negative.
         */
        IDynamicVirtualQueue(float max_queue_size)
        {
            if (max_queue_size < 0.0f)
            {
                throw std::invalid_argument("Tried to initiate queue with negative maximum size");
            }
            max_queue_size_ = max_queue_size;
        };

        /**
         * @brief Get the size of the queue.
         * @return Size of the queue.
         */
        virtual float getSize()=0;

        /**
         * @brief Updates the queue by the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param change Change in the queue that wants to be added to the queue
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(const float change)=0;
        
        /**
         * @brief Indicates if the queue is full.
         * @return Boolean if the queue is full or not.
         */
        virtual bool isfull()=0;

    protected:
        /**
         * @brief Bound the maximum size of a queue.
         **/
        float max_queue_size_=0;
};