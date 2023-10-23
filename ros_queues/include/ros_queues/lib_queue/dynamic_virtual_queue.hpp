#pragma once

#include <mutex>

#include "ros_queues/lib_queue/I_dynamic_queue.hpp"
#include "ros_queues/lib_queue/virtual_queue.hpp"


/**
 * @brief Virtual queue with interfaces to affect its dynamic. Implemented like virtual queues used with inequality constraint where their size can't go below 0. 
 */
class InConVirtualQueue: public IDynamicQueue<VirtualQueue>
{
    public:
        InConVirtualQueue(int max_queue_size): IDynamicQueue(max_queue_size) {};
        
        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getSize() override;

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods InConVirtualQueue::arrival_prediction and InConVirtualQueue::transmission_prediction.
         * @throw Throws a NegativeArrivalPredictionException if the IDynamicQueue::arrival_prediction predicts an negative number of incoming elements which sould never logicaly happen.
         * @throw Throws a NegativeDeparturePredictionException if the IDynamicQueue::transmission_prediction predicts an negative number of departing elements which sould never logicaly happen.
         * @return Predicted size of the queue after evaluation.
         */
        virtual int evaluate() override;
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param arriving_elements Virtual queue to append to the internal virtual queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(VirtualQueue arriving_elements, const int nb_departing_elements) override;
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param nb_arriving_elements Number of elements to add to queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(const int nb_arriving_elements, const int nb_departing_elements);

    protected:
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. Override this method to define a specific arrival prediction behavior.
         * @return Returns the number of elements that could be added to the queue.
         */
        virtual int arrival_prediction() override;

        /**
         * @brief Method used in the evaluation process to predict what will be the departure size. Override this method to define a specific transmission prediction behavior.
         * @return Returns the number of elements that could be removed from the queue.
         */
        virtual int transmission_prediction() override;

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        std::mutex queue_manipulation_mutex_;
};

/**
 * @brief Virtual queue with interfaces to affect its dynamic. Implemented like virtual queues used with equality constraint where their size can go below 0. 
 */
class EqConVirtualQueue: public IDynamicQueue<NVirtualQueue>
{
    public:
        EqConVirtualQueue(int max_queue_size): IDynamicQueue(max_queue_size) {};
        /**
         * @brief Get the size of the internal queue.
         * @return Return the size of the internal queue.
         */
        virtual int getSize() override;

        /**
         * @brief Evaluate the size of the queue based on the real size of the queue and predicted arrival and departure rate.
         * @details It evaluates the size of the queue based on it's actual size and predicted arrival and departure rate. Those prediction are evaluated based on the methods InConVirtualQueue::arrival_prediction and InConVirtualQueue::transmission_prediction.
         * @throw Throws a NegativeArrivalPredictionException if the IDynamicQueue::arrival_prediction predicts an negative number of incoming elements which sould never logicaly happen.
         * @throw Throws a NegativeDeparturePredictionException if the IDynamicQueue::transmission_prediction predicts an negative number of departing elements which sould never logicaly happen.
         * @return Predicted size of the queue after evaluation.
         */
        virtual int evaluate() override;
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param arriving_elements Virtual queue to append to the internal virtual queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(NVirtualQueue arriving_elements, const int nb_departing_elements) override;
        
        /**
         * @brief Updates the queue by adding the arriving_elements and by removing the specifiy number of departing elements while respecting the maximum queue size.
         * @param nb_arriving_elements Number of elements to add to queue.
         * @param nb_departing_elements Number of elements to remove from the queue.
         * @throw Throws an invalid_argument exception if the departure arguement is negative since it can't transmit negative number of elements.
         * @return Boolean of it the queue overflowed while adding elements.
         */
        virtual bool update(const int nb_arriving_elements, const int nb_departing_elements);

    protected:
        /**
         * @brief Method used in the evaluation process to predict what will be the arrival size. Override this method to define a specific arrival prediction behavior.
         * @return Returns the number of elements that could be added to the queue.
         */
        virtual int arrival_prediction() override;

        /**
         * @brief Method used in the evaluation process to predict what will be the departure size. Override this method to define a specific transmission prediction behavior.
         * @return Returns the number of elements that could be removed from the queue.
         */
        virtual int transmission_prediction() override;

        /**
         * @brief Mutex to protect access to the internal queue.
         */
        std::mutex queue_manipulation_mutex_;
};