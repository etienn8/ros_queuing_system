#include <stdexcept>
#include <deque>

#include <gtest/gtest.h>

#include "include/dynamic_queues_test.hpp"
#include "include/trajectory.hpp"

#include "ros_queue/lib_queue/dynamic_queue.hpp"
#include "ros_queue/lib_queue/dynamic_converted_queue.hpp"
#include "ros_queue/lib_queue/queue_exception.hpp"


using std::list;
using std::deque;

static const list<Position3D> point_list_0 = {Position3D(1,1,1), Position3D(2,2,2), Position3D(3,3,3)};
static const list<Position3D> point_list_1 = {Position3D(4,4,4), Position3D(5,5,5), Position3D(6,6,6), Position3D(7,7,7)};

// Conversion function
void traj_to_byte_conversion(deque<Trajectory>&& queue_trajectory, deque<ElementWithConvertedSize<Trajectory>>& converted_queue)
{
    for(typename deque<Trajectory>::iterator it = queue_trajectory.begin(); it != queue_trajectory.end(); ++it)
    {
        int converted_size = it->reference_frame_.size() + it->point_list_.size()*sizeof(Position3D);

        ElementWithConvertedSize<Trajectory> convertedElement(std::move(*it), converted_size);
        converted_queue.push_back(std::move(convertedElement));
    }
}

void no_conversion(deque<Trajectory>&& queue_trajectory, deque<ElementWithConvertedSize<Trajectory>>& converted_queue) {};

void null_size_conversion(deque<Trajectory>&& queue_trajectory, deque<ElementWithConvertedSize<Trajectory>>& converted_queue)
{
    for(typename deque<Trajectory>::iterator it = queue_trajectory.begin(); it != queue_trajectory.end(); ++it)
    {
        int converted_size = 0;

        ElementWithConvertedSize<Trajectory> convertedElement(std::move(*it), converted_size);
        converted_queue.push_back(std::move(convertedElement));
    }
}

void traj_to_negative_conversion(deque<Trajectory>&& queue_trajectory, deque<ElementWithConvertedSize<Trajectory>>& converted_queue)
{
    for(typename deque<Trajectory>::iterator it = queue_trajectory.begin(); it != queue_trajectory.end(); ++it)
    {
        int converted_size = it->reference_frame_.size() + it->point_list_.size()*sizeof(Position3D);

        ElementWithConvertedSize<Trajectory> convertedElement(std::move(*it), -converted_size);
        converted_queue.push_back(std::move(convertedElement));
    }
}

// Tests for the dynamic queue
TEST(DynamicQueueTest, manipultationTest)
{
    // Initialize queue
    DynamicQueue<int> q(10);

    // Initialize queue to inject
    deque<int> arrival_queue = {1, 2, 3, 4, 5};
    const deque<int> empty_deque;
    deque<int> temp_deque = arrival_queue;

    // Add some elements and verify expected size and content
    q.update(arrival_queue,0);
    EXPECT_EQ(q.getSize(), 5);
    EXPECT_EQ(q.getInternalQueue(), temp_deque);

    q.update(arrival_queue,0);
    EXPECT_EQ(q.getSize(), 10);
    temp_deque = {1, 2, 3, 4, 5, 1, 2, 3, 4, 5};
    EXPECT_EQ(q.getInternalQueue(), temp_deque);

    // Remove some elements
    q.update(empty_deque, 6);
    EXPECT_EQ(q.getSize(), 4);
    temp_deque = {2, 3, 4, 5};
    EXPECT_EQ(q.getInternalQueue(), temp_deque);

    // Add and remove elements
    q.update(arrival_queue, 3);
    EXPECT_EQ(q.getSize(), 6);
    temp_deque = {5, 1, 2, 3, 4, 5};
    EXPECT_EQ(q.getInternalQueue(), temp_deque);

    // Remove negative number
    EXPECT_THROW(q.update(empty_deque,-1),invalid_argument);
}

TEST(DynamicQueueTest, initTest)
{
    // Verify if throws an error with a negative maximum size
    const int QUEUE_MAX_SIZE = -10;
    EXPECT_THROW(DynamicQueue<int> q(QUEUE_MAX_SIZE), invalid_argument);
}

TEST(DynamicQueueTest, dimensionLimitTest)
{
    const int queue_max_size = 10;
    // Initialize queue
    DynamicQueue<int> q(10);

    // Initialize queue to inject
    deque<int> arrival_queue = {1, 2, 3, 4, 5};
    deque<int> big_arrival_queue = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    const deque<int> empty_deque;

    // Verify lower bound
    q.update(empty_deque, 1);
    EXPECT_EQ(q.getSize(), 0);

    // Verify upper bound
    q.update(big_arrival_queue, 0);
    deque<int> temp_deque = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    EXPECT_EQ(q.getSize(), queue_max_size);
    EXPECT_EQ(q.getInternalQueue(), temp_deque);

    //Verify manipulation near upper bound
     q.update(arrival_queue, 6);
     temp_deque = {12, 13, 14, 15, 1, 2, 3, 4, 5};
    EXPECT_EQ(q.getSize(), 9);
    EXPECT_EQ(q.getInternalQueue(), temp_deque);
}

TEST(DynamicQueueTest, predictionTest)
{
    const int queue_max_size = 10;

    // Initialize queue with an implementation of its predicted values
    DynamicQueueMockedPrediction<int> q(10);
    deque<int> arrival_queue = {1, 2, 3, 4, 5};
    q.update(arrival_queue, 0);

    // Initialize arrival rate and transmission
    q.predicted_arrival_ = 4;
    q.predicted_transmission_ = 2; 

    // Evaluate queue and verify its content and size
    EXPECT_EQ(q.evaluate(), 7);
    // Real size should not have change after the evaluation
    EXPECT_EQ(q.getSize(), 5);

    // Verify the upper bound of evaluation
    q.predicted_arrival_ = 10;
    q.predicted_transmission_ = 0;

    EXPECT_EQ(q.evaluate(), 10);

    // Verify the lower bound of evaluation
    q.predicted_arrival_ = 0;
    q.predicted_transmission_ = 10;

    EXPECT_EQ(q.evaluate(), 0);

    // Verify illegal predicted values
    q.predicted_arrival_ = -1;
    q.predicted_transmission_ = 10;
    EXPECT_THROW(q.evaluate(), NegativeArrivalPredictionException);

    q.predicted_arrival_ = 0;
    q.predicted_transmission_ = -1;
    EXPECT_THROW(q.evaluate(), NegativeDeparturePredictionException);
}

TEST(DynamicQueueTest, specializedPredictionTest)
{
    const int queue_max_size = 10;

    // Initialize queue with an evaluation that takes an in at the input
    SpecializedDynamicQueueMockedPrediction<int, int> q(10);
    deque<int> arrival_queue = {1, 2, 3, 4, 5};
    q.update(arrival_queue, 0);

    int state = 4;
    // Evaluate queue and verify its content and size. This implementation predicts an arrival of 4+1 and transmission of 4.
    EXPECT_EQ(q.evaluate(state), 6);
}

TEST(DynamicQueueTest, transmissionTest)
{
    // Create a mock to receive data
    DynamicQueueMockedTransmission<int> q(10);
    deque<int> arrival_queue = {1, 2, 3, 4, 5};

    // Fill queue with data
    q.update(arrival_queue, 2);

    // Verify transmission of empty queue
    EXPECT_EQ(q.transmittedElements.size(), 0);

    // Transmit data to mock and verify content
    deque<int> temp_deque = {1,2,3};

    q.update(arrival_queue, 3);
    EXPECT_EQ(q.transmittedElements.size(), 3);
    EXPECT_EQ(q.transmittedElements, temp_deque);
}


// Tests for the queue with size conversion
TEST(DynamicConvertedQueueTest, manipultationTest)
{
    // Initialize queue
    const int queue_max_size = 512;
    DynamicConvertedQueueWithFPtr<Trajectory> q(queue_max_size, traj_to_byte_conversion);
    // Initialize queue to inject
    // Size of "frame_X" = 7 bytes and each point list contains 3 int (3*4 byte = 12). So each trajectory is 7 + 12*point_in_point_list.
    // The frame_0 and frame_2 trajectories take 43 bytes each and the frame_1 trajectory takes 55 bytes
    deque<Trajectory> arrival_queue = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};
    const deque<Trajectory> empty_deque;
    deque<Trajectory> temp_deque = arrival_queue;

    // Add some elements and verify expected size and content
    q.update(arrival_queue,0);
    EXPECT_EQ(q.getSize(), 141);

    EXPECT_EQ(q.getInternalQueue(), temp_deque);

    q.update(arrival_queue,0);
    EXPECT_EQ(q.getSize(), 282);
    temp_deque = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0), Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};
    EXPECT_EQ(q.getInternalQueue(), temp_deque);

    // Remove some elements
    q.update(empty_deque, 2);
    EXPECT_EQ(q.getSize(), 184);
    temp_deque = {Trajectory("frame_2", point_list_0), Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};
    EXPECT_EQ(q.getInternalQueue(), temp_deque);

    // Add and remove elements
    q.update(arrival_queue, 2);
    EXPECT_EQ(q.getSize(), 239);
    temp_deque = {Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0), Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};
    EXPECT_EQ(q.getInternalQueue(), temp_deque);

    // Remove negative number
    EXPECT_THROW(q.update(empty_deque,-1),invalid_argument);
}

TEST(DynamicConvertedQueueTest, initTest)
{
    // Verify if throws an error with a negative maximum size
    const int QUEUE_MAX_SIZE = -10;
    EXPECT_THROW(DynamicConvertedQueueWithFPtr<Trajectory> q(-512, traj_to_byte_conversion), invalid_argument);
}

TEST(DynamicConvertedQueueTest, dimensionLimitTest)
{
    // Initialize queue
    const int queue_max_size = 256;
    DynamicConvertedQueueWithFPtr<Trajectory> q(queue_max_size, traj_to_byte_conversion);

    // Initialize queue to inject
    deque<Trajectory> arrival_queue = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};
    const deque<Trajectory> empty_deque;

    // Verify lower bound
    q.update(empty_deque, 1);
    EXPECT_EQ(q.getSize(), 0);

    // Verify upper bound
    q.update(arrival_queue, 0);
    q.update(arrival_queue, 0);
    deque<Trajectory> temp_deque = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0), Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1)};
    EXPECT_EQ(q.getSize(), 239);
    EXPECT_EQ(q.getInternalQueue(), temp_deque);
}

TEST(DynamicConvertedQueueTest, predictionTest)
{
    const int queue_max_size = 512;

    // Initialize queue with an implementation of its predicted values
    DynamicConvertedQueueMockedPrediction<Trajectory> q(queue_max_size, traj_to_byte_conversion);

    // Initialize queue to inject
    deque<Trajectory> arrival_queue = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};
    const deque<Trajectory> empty_deque;

    q.update(arrival_queue, 0);

    // Initialize arrival rate and transmission
    q.predicted_arrival_ = 128;
    q.predicted_transmission_ = 1; 

    // Evaluate queue and verify its content and size
    EXPECT_EQ(q.evaluate(), 226);
    // Real size should not have change after the evaluation
    EXPECT_EQ(q.getSize(), 141);

    // Verify the upper bound of evaluation
    q.predicted_arrival_ = 400;
    q.predicted_transmission_ = 0; 
    
    EXPECT_EQ(q.evaluate(), queue_max_size);

    // Verify the lower bound of evaluation
    q.predicted_arrival_ = 0;
    q.predicted_transmission_ = 4;

    EXPECT_EQ(q.evaluate(), 0);

    // Verify illegal predicted values
    q.predicted_arrival_ = -1;
    q.predicted_transmission_ = 0;
    EXPECT_THROW(q.evaluate(), NegativeArrivalPredictionException);

    q.predicted_arrival_ = 0;
    q.predicted_transmission_ = -1;
    EXPECT_THROW(q.evaluate(), NegativeDeparturePredictionException);
}

TEST(DynamicConvertedQueueTest, specializedPredictionTest)
{
    const int queue_max_size = 512;

    // Initialize queue with an evaluation that takes an in at the input
    SpecializedDynamicConvertedQueueMockedPrediction<Trajectory, int> q(queue_max_size, traj_to_byte_conversion);
    deque<Trajectory> arrival_queue = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};
    q.update(arrival_queue, 0);
    q.update(arrival_queue, 0);

    int state = 4;
    // Evaluate queue and verify its content and size. This implementation predicts an arrival of 4+1 and transmission of 4.
    EXPECT_EQ(q.evaluate(state), 103);
}

TEST(DynamicConvertedQueueTest, transmissionTest)
{
    const int queue_max_size = 512;

    // Create a mock to receive data
    DynamicConvertedQueueMockedTransmission<Trajectory> q(queue_max_size, traj_to_byte_conversion);
    deque<Trajectory> arrival_queue = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};
   
    // Fill queue with data
    q.update(arrival_queue, 2);

    // Verify transmission of empty queue
    EXPECT_EQ(q.transmittedElements.size(), 0);

    // Transmit data to mock and verify content
    deque<Trajectory> temp_deque = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1)};

    q.update(arrival_queue, 2);
    EXPECT_EQ(q.transmittedElements.size(), 2);
    EXPECT_EQ(q.transmittedElements, temp_deque);
}

TEST(DynamicConvertedQueueTest, badConversionTest)
{
    //Initialize queue
    const int queue_max_size = 256;

    // Initialize queue to inject
    deque<Trajectory> arrival_queue = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};
    
    // Verify that null conversion function is detected
    DynamicConvertedQueueWithFPtr<Trajectory> q2(queue_max_size, null_size_conversion);
    EXPECT_THROW(q2.update(arrival_queue, 0), BadConversionException);

    // Verify that size of the converted list is the same as the arriving elements
    DynamicConvertedQueueWithFPtr<Trajectory> q3(queue_max_size, no_conversion);
    EXPECT_THROW(q3.update(arrival_queue, 0), BadConversionException);

    //Verify that conversion can't give a negative number
    DynamicConvertedQueueWithFPtr<Trajectory> q4(queue_max_size, traj_to_negative_conversion);
    EXPECT_THROW(q4.update(arrival_queue, 0), BadConversionException);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


