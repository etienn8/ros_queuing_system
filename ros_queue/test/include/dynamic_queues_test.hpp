#pragma once

#include <deque>
#include <list>
#include <string>

#include "ros_queue/lib_queue/dynamic_queue.hpp"

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

    virtual bool transmit(deque<TQueueElementType> &queue_to_transmit)
    {   
        //Receiving data
        while(!queue_to_transmit.empty())
        {
            transmittedElements.push_back(queue_to_transmit.front());
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


template<typename TQueueElementType> 
class DynamicConvertedQueueMockedPrediction: public DynamicConvertedQueue<TQueueElementType>
{
    public:
        DynamicConvertedQueueMockedPrediction(int max_queue_size, void (*conversionFunction)(deque<TQueueElementType>&,
                                                deque<ElementWithConvertedSize<TQueueElementType>>&)):
                                                DynamicConvertedQueue<TQueueElementType>(max_queue_size,conversionFunction) {};

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
class SpecializedDynamicConvertedQueueMockedPrediction: public DynamicConvertedQueue<TQueueElementType, TStates>
{
    public:
        SpecializedDynamicConvertedQueueMockedPrediction(int max_queue_size, void (*conversionFunction)(deque<TQueueElementType>&,
                                                deque<ElementWithConvertedSize<TQueueElementType>>&)):
                                                DynamicConvertedQueue<TQueueElementType, TStates>(max_queue_size,conversionFunction) {};

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
class DynamicConvertedQueueMockedTransmission: public DynamicConvertedQueue<TQueueElementType>
{
    public:
        DynamicConvertedQueueMockedTransmission(int max_queue_size, void (*conversionFunction)(deque<TQueueElementType>&,
                                                deque<ElementWithConvertedSize<TQueueElementType>>&)):
                                                DynamicConvertedQueue<TQueueElementType>(max_queue_size,conversionFunction) {};

        deque<TQueueElementType> transmittedElements;

    virtual bool transmit(deque<TQueueElementType> &queue_to_transmit)
    {   
        //Receiving data
        while(!queue_to_transmit.empty())
        {
            transmittedElements.push_back(queue_to_transmit.front());
            queue_to_transmit.pop_front();
        }

        return 1;
    }
};

class Position3D
{
    public:
        Position3D(int x, int y, int z): x_(x), y_(y), z_(z) {};

        int x_ = 0;
        int y_ = 0;
        int z_ = 0;

        friend bool operator==(const Position3D& traj1, const Position3D& traj2)
        {
            return (traj1.x_ == traj2.x_) && (traj1.y_ == traj2.y_) && (traj1.z_ == traj2.z_);
        }
};

class Trajectory
{
    public:
        Trajectory(string reference_frame, list<Position3D> point_list): 
                    reference_frame_(reference_frame), point_list_(point_list) {};

        list<Position3D> point_list_;
        string reference_frame_;

        friend bool operator==(const Trajectory& traj1, const Trajectory& traj2)
        {
            
            return (traj1.reference_frame_==traj2.reference_frame_) && (traj1.point_list_==traj2.point_list_);
        }

};

