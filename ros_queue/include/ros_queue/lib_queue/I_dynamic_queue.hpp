#pragma once

#include <stdexcept>

using std::invalid_argument;

/**
 * @brief Interface class that wraps a type of queue with interfaces to interact with it. 
 * @tparam TQueueType Type of queue (example: std::queue or std::dequeu).
 */
template<typename TQueueType, typename TStates=void>
class IDynamicQueue
{
    public:
        /**
         * @brief Constructor where the max_queue_size is initialized and checked. 
         * @param max_queue_size Maximum size that the queue can reach.
         * @throw Throws an invalid_argument if the specified queue_size is negative.
         */
        IDynamicQueue(int max_queue_size)
        {
            if (max_queue_size < 0)
            {
                throw invalid_argument("Tried to initiate queue with negative maximum size");
            }
            max_queue_size_ = max_queue_size;
        };

        /**
         * @brief Get the size of the queue.
         * @return Size of the queue.
         */
        virtual int getSize(){return internal_queue_.size();};

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods IDynamicQueue::arrival_prediction and IDynamicQueue::transmission_prediction.
         * @return Predicted size of the queue after evaluation.
         */
        virtual int evaluate(const TStates& states)=0;
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removind the specifiy number of departing elements while respecting the maximum queue size.
         * @param arriving_elements Queue of elements to add to the queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(TQueueType arriving_elements, const int nb_departing_elements)=0;
        
        /**
         * @brief Indicates if the queue is full.
         * @return Boolean if the queue is full or not.
         */
        inline virtual bool isfull(){return internal_queue_.size()==max_queue_size_;};

    protected:
        /**
         * @brief Bound the maximum size of a queue.
         **/
        int max_queue_size_=0;

        /**
         * @brief Queue object used internaly.
         **/
        TQueueType internal_queue_;

        /**
         * @brief Method to override that predicts the size of incoming elements.
         * @return Return an integer that predicts how many element would be added to the queue.
         */
        virtual int arrival_prediction(const TStates& states)=0;

        /**
         * @brief Method to override that predicts the number of elements that could leave the queue.
         * @return Return an integer that predicts how many element would be removed from the queue.
         */
        virtual int transmission_prediction(const TStates& states)=0;
};

/**
 * @brief Interface class that wraps a type of queue with interfaces to interact with it. 
 * @tparam TQueueType Type of queue (example: std::queue or std::dequeu).
 */
template<typename TQueueType>
class IDynamicQueue<TQueueType, void>
{
    public:
        /**
         * @brief Constructor where the max_queue_size is initialized and checked. 
         * @param max_queue_size Maximum size that the queue can reach.
         * @throw Throws an invalid_argument if the specified queue_size is negative.
         */
        IDynamicQueue(int max_queue_size)
        {
            if (max_queue_size < 0)
            {
                throw invalid_argument("Tried to initiate queue with negative maximum size");
            }
            max_queue_size_ = max_queue_size;
        };

        /**
         * @brief Get the size of the queue.
         * @return Size of the queue.
         */
        virtual int getSize(){return internal_queue_.size();};

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods IDynamicQueue::arrival_prediction and IDynamicQueue::transmission_prediction.
         * @return Predicted size of the queue after evaluation.
         */
        virtual int evaluate(void)=0;
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removind the specifiy number of departing elements while respecting the maximum queue size.
         * @param arriving_elements Queue of elements to add to the queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(TQueueType arriving_elements, const int nb_departing_elements)=0;
        
        /**
         * @brief Indicates if the queue is full.
         * @return Boolean if the queue is full or not.
         */
        inline virtual bool isfull(){return internal_queue_.size()==max_queue_size_;};

    protected:
        /**
         * @brief Bound the maximum size of a queue.
         **/
        int max_queue_size_=0;

        /**
         * @brief Queue object used internaly.
         **/
        TQueueType internal_queue_;

        /**
         * @brief Method to override that predicts the size of incoming elements.
         * @return Return an integer that predicts how many element would be added to the queue.
         */
        virtual int arrival_prediction(void)=0;

        /**
         * @brief Method to override that predicts the number of elements that could leave the queue.
         * @return Return an integer that predicts how many element would be removed from the queue.
         */
        virtual int transmission_prediction(void)=0;
};