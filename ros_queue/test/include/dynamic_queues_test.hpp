#pragma once

#include <deque>
#include <list>
#include <string>

#include "ros_queue/lib_queue/dynamic_queue.hpp"
#include "ros_queue/lib_queue/dynamic_converted_queue.hpp"


using namespace std;

template<typename TQueueElementType> 
class DynamicQueueMockedPrediction: public DynamicQueue<TQueueElementType>
{
    public:
        DynamicQueueMockedPrediction(int max_queue_size): DynamicQueue<TQueueElementType>(max_queue_size) {};

        int predicted_arrival_ = 0;
        int predicted_transmission_ = 0;

    protected:
        virtual int arrival_prediction() override
        {
            return predicted_arrival_;
        };

        virtual int transmission_prediction() override
        {
            return predicted_transmission_;
        };
};

template<typename TQueueElementType> 
class DynamicQueueMockedTransmission: public DynamicQueue<TQueueElementType>
{
    public:
        DynamicQueueMockedTransmission(int max_queue_size): DynamicQueue<TQueueElementType>(max_queue_size) {};

        deque<TQueueElementType> transmittedElements;

    virtual bool transmit(deque<TQueueElementType>&& queue_to_transmit) override
    {   
        //Receiving data
        while(!queue_to_transmit.empty())
        {
            transmittedElements.push_back(std::move(queue_to_transmit.front()));
            queue_to_transmit.pop_front();
        }

        return 1;
    }
};

template<typename TQueueElementType, typename TStates> 
class SpecializedDynamicQueueMockedPrediction: public DynamicQueue<TQueueElementType, TStates>
{
    public:
        SpecializedDynamicQueueMockedPrediction(int max_queue_size): DynamicQueue<TQueueElementType, TStates>(max_queue_size) {};

    protected:
        virtual int arrival_prediction(const TStates& states) override
        {
            return states+1;
        };

        virtual int transmission_prediction(const TStates& states) override
        {
            return states;
        };
};

template<typename TQueueElementType, typename TStates=void> 
class DynamicConvertedQueueWithFPtr : public DynamicConvertedQueue<TQueueElementType, TStates>
{
        public:
        DynamicConvertedQueueWithFPtr(int max_queue_size, void (*conversionFunction)(deque<TQueueElementType>&&,
                                                deque<ElementWithConvertedSize<TQueueElementType>>&)):
                                                DynamicConvertedQueue<TQueueElementType, TStates>(max_queue_size), generateConvertedQueueMock_(conversionFunction) {};

    protected:

        virtual void generateConvertedQueue(deque<TQueueElementType>&& arriving_queue, deque<ElementWithConvertedSize<TQueueElementType>>& converted_dequeue) override
        {
            if (generateConvertedQueueMock_)
            {
                generateConvertedQueueMock_(std::move(arriving_queue), converted_dequeue);
            }
        }
    private:
        void (*generateConvertedQueueMock_)(deque<TQueueElementType>&&, deque<ElementWithConvertedSize<TQueueElementType>>&);
};

template<typename TQueueElementType> 
class DynamicConvertedQueueWithFPtr<TQueueElementType, void>: public DynamicConvertedQueue<TQueueElementType>
{
        public:
        DynamicConvertedQueueWithFPtr(int max_queue_size, void (*conversionFunction)(deque<TQueueElementType>&&,
                                                deque<ElementWithConvertedSize<TQueueElementType>>&)):
                                                DynamicConvertedQueue<TQueueElementType>(max_queue_size), 
                                                generateConvertedQueueMock_(conversionFunction){};

        void (*generateConvertedQueueMock_)(deque<TQueueElementType>&&, deque<ElementWithConvertedSize<TQueueElementType>>&);

    protected:

        virtual void generateConvertedQueue(deque<TQueueElementType>&& arriving_queue, deque<ElementWithConvertedSize<TQueueElementType>>& converted_queue) override
        {
            if (generateConvertedQueueMock_)
            {
                generateConvertedQueueMock_(std::move(arriving_queue), converted_queue);
            }
        }
};


template<typename TQueueElementType> 
class DynamicConvertedQueueMockedPrediction: public DynamicConvertedQueueWithFPtr<TQueueElementType>
{
    public:
        DynamicConvertedQueueMockedPrediction(int max_queue_size, void (*conversionFunction)(deque<TQueueElementType>&&,
                                                deque<ElementWithConvertedSize<TQueueElementType>>&)):
                                                DynamicConvertedQueueWithFPtr<TQueueElementType>(max_queue_size,conversionFunction) {};

        int predicted_arrival_ = 0;
        int predicted_transmission_ = 0;

    protected:
        virtual int arrival_prediction() override
        {
            return predicted_arrival_;
        };

        virtual int transmission_prediction() override
        {
            return predicted_transmission_;
        };
};

template<typename TQueueElementType, typename TStates> 
class SpecializedDynamicConvertedQueueMockedPrediction: public DynamicConvertedQueueWithFPtr<TQueueElementType, TStates>
{
    public:
        SpecializedDynamicConvertedQueueMockedPrediction(int max_queue_size, void (*conversionFunction)(deque<TQueueElementType>&&,
                                                deque<ElementWithConvertedSize<TQueueElementType>>&)):
                                                DynamicConvertedQueueWithFPtr<TQueueElementType, TStates>(max_queue_size,conversionFunction) {};

    protected:
        virtual int arrival_prediction(const TStates& states) override
        {
            return states+1;
        };

        virtual int transmission_prediction(const TStates& states) override
        {
            return states;
        };
};

template<typename TQueueElementType> 
class DynamicConvertedQueueMockedTransmission: public DynamicConvertedQueueWithFPtr<TQueueElementType>
{
    public:
        DynamicConvertedQueueMockedTransmission(int max_queue_size, void (*conversionFunction)(deque<TQueueElementType>&&,
                                                deque<ElementWithConvertedSize<TQueueElementType>>&)):
                                                DynamicConvertedQueueWithFPtr<TQueueElementType>(max_queue_size,conversionFunction) {};

        deque<TQueueElementType> transmittedElements;

    virtual bool transmit(deque<TQueueElementType>&& queue_to_transmit) override
    {   
        //Receiving data
        while(!queue_to_transmit.empty())
        {
            transmittedElements.push_back(std::move(queue_to_transmit.front()));
            queue_to_transmit.pop_front();
        }

        return 1;
    }
};
