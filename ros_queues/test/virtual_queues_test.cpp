#include <stdexcept>

#include <gtest/gtest.h>

#include "ros_queues/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queues/lib_queue/virtual_queue.hpp"

#include "ros_queues/lib_queue/queue_exception.hpp"


using std::invalid_argument;

class InConVirtualQueueEvaluation: public InConVirtualQueue
{
    public:
        InConVirtualQueueEvaluation(int max_queue_size): InConVirtualQueue(max_queue_size){};
        int predicted_arrival_=0;
        int predicted_transmission_=0;

    protected:
        virtual int arrival_prediction() override
        {
            return predicted_arrival_;
        }
        
        virtual int transmission_prediction() override
        {
            return predicted_transmission_;
        }
};


class EqConVirtualQueueEvaluation: public EqConVirtualQueue
{
    public:
        EqConVirtualQueueEvaluation(int max_queue_size): EqConVirtualQueue(max_queue_size){};
        int predicted_arrival_=0;
        int predicted_transmission_=0;

    protected:
        virtual int arrival_prediction() override
        {
            return predicted_arrival_;
        }
        
        virtual int transmission_prediction() override
        {
            return predicted_transmission_;
        }
};


TEST(VirtualQueueTest, manipulationTest)
{
    VirtualQueue vq;

    EXPECT_EQ(vq.size(), 0);
    vq.push(2);
    EXPECT_EQ(vq.size(), 2);
    vq.push(2);
    EXPECT_EQ(vq.size(), 4);

    vq.pop(2);
    EXPECT_EQ(vq.size(), 2);
    vq.pop(2);
    EXPECT_EQ(vq.size(), 0);

    vq.setSize(5);
    EXPECT_EQ(vq.size(), 5);

    EXPECT_THROW(vq.push(-7),invalid_argument);
    EXPECT_THROW(vq.pop(-7),invalid_argument);

    EXPECT_THROW(vq.setSize(-1), invalid_argument);
}

TEST(VirtualQueueTest, dimensionLimitTest)
{
    VirtualQueue vq;

    EXPECT_EQ(vq.empty(), true);
    vq.push(2);
    EXPECT_EQ(vq.empty(), false);

    vq.pop(4);
    EXPECT_EQ(vq.size(), 0);
    vq.pop(2);
    EXPECT_EQ(vq.size(), 0);
}

TEST(NVirtualQueueTest, manipulationTest)
{
    NVirtualQueue vq;

    EXPECT_EQ(vq.size(), 0);
    vq.push(2);
    EXPECT_EQ(vq.size(), 2);
    vq.push(2);
    EXPECT_EQ(vq.size(), 4);

    vq.pop(2);
    EXPECT_EQ(vq.size(), 2);
    vq.pop(2);
    EXPECT_EQ(vq.size(), 0);
    vq.pop(3);
    EXPECT_EQ(vq.size(), -3);

    vq.setSize(5);
    EXPECT_EQ(vq.size(), 5);

    EXPECT_THROW(vq.push(-7),invalid_argument);
    EXPECT_THROW(vq.pop(-7),invalid_argument);
    
    vq.setSize(-1);
    EXPECT_EQ(vq.size(), -1);
}

TEST(NVirtualQueueTest, dimensionLimitTest)
{
    NVirtualQueue vq;

    EXPECT_EQ(vq.empty(), true);
    vq.push(2);
    EXPECT_EQ(vq.empty(), false);
    vq.setSize(-2);
    EXPECT_EQ(vq.empty(), false);
}

// Test for the inequqality constraint virtual queue
TEST(InConVirutalQueueDynamicTest, manipultationTest)
{
    InConVirtualQueue vq(10);

    VirtualQueue vq_arrival;
    vq_arrival.setSize(5);

    EXPECT_EQ(vq.getSize(), 0);
    vq.update(4,3);
    EXPECT_EQ(vq.getSize(), 4);
    vq.update(4,3);
    EXPECT_EQ(vq.getSize(), 5);

    vq.update(vq_arrival, 2);
    EXPECT_EQ(vq.getSize(), 8);

    EXPECT_THROW(vq.update(-1,0),invalid_argument);
    EXPECT_THROW(vq.update(0,-1),invalid_argument);
    EXPECT_THROW(vq.update(vq_arrival,-1),invalid_argument);
}

TEST(InConVirutalQueueDynamicTest, initTest)
{
    const int QUEUE_MAX_SIZE = -10;
    EXPECT_THROW(InConVirtualQueue vq(QUEUE_MAX_SIZE), invalid_argument);
}

TEST(InConVirutalQueueDynamicTest, dimensionLimitTest)
{
    const int QUEUE_MAX_SIZE = 10;
    InConVirtualQueue vq(QUEUE_MAX_SIZE);
    
    VirtualQueue vq_arrival;
    vq_arrival.setSize(QUEUE_MAX_SIZE+1);

    vq.update(0, 1);
    EXPECT_EQ(vq.getSize(), 0);
    vq.update(QUEUE_MAX_SIZE + 1, 0);
    EXPECT_EQ(vq.getSize(), QUEUE_MAX_SIZE);

    vq.update(vq_arrival, 0);
    EXPECT_EQ(vq.getSize(), QUEUE_MAX_SIZE);

    vq.update(2,3);
    EXPECT_EQ(vq.getSize(), 9);
}

TEST(InConVirutalQueueDynamicTest, predictionTest)
{
    const int QUEUE_MAX_SIZE = 10;
    InConVirtualQueueEvaluation vq(QUEUE_MAX_SIZE);
    vq.predicted_arrival_ = 2;
    vq.predicted_transmission_ = 3;

    vq.update(4,0);
    EXPECT_EQ(vq.evaluate(), 4 - vq.predicted_transmission_ + vq.predicted_arrival_);
    EXPECT_EQ(vq.getSize(), 4);

    vq.predicted_arrival_ = 7;
    vq.predicted_transmission_ = 0;
    EXPECT_EQ(vq.evaluate(), QUEUE_MAX_SIZE);

    vq.predicted_arrival_ = -1;
    EXPECT_THROW(vq.evaluate(), NegativeArrivalPredictionException);
    
    vq.predicted_arrival_ = 0;
    vq.predicted_transmission_ = -1;
    EXPECT_THROW(vq.evaluate(), NegativeDeparturePredictionException);
}

TEST(InConVirutalQueueDynamicTest, memorySizeTest)
{
    const int QUEUE_MAX_SIZE = 10;
    InConVirtualQueue vq(QUEUE_MAX_SIZE);

    EXPECT_EQ(vq.getMemSize(), sizeof(int));
}

// Test for the equality constraint virtual queue
TEST(EqConVirtualQueueDynamicTest, manipultationTest)
{
    EqConVirtualQueue vq(10);

    NVirtualQueue vq_arrival;
    vq_arrival.setSize(5);

    EXPECT_EQ(vq.getSize(), 0);
    vq.update(4,3);
    EXPECT_EQ(vq.getSize(), 1);
    vq.update(4,3);
    EXPECT_EQ(vq.getSize(), 2);

    vq.update(vq_arrival, 2);
    EXPECT_EQ(vq.getSize(), 5);

    vq.update(0,9);
    EXPECT_EQ(vq.getSize(), -4);

    EXPECT_THROW(vq.update(-1,0),invalid_argument);
    EXPECT_THROW(vq.update(0,-1),invalid_argument);
    EXPECT_THROW(vq.update(vq_arrival,-1),invalid_argument);
}

TEST(EqConVirtualQueueDynamicTest, initTest)
{
    const int QUEUE_MAX_SIZE = -10;
    EXPECT_THROW(EqConVirtualQueue vq(QUEUE_MAX_SIZE), invalid_argument);
}

TEST(EqConVirtualQueueDynamicTest, dimensionLimitTest)
{
    const int QUEUE_MAX_SIZE = 10;
    EqConVirtualQueue vq(QUEUE_MAX_SIZE);
    
    NVirtualQueue vq_arrival;
    vq_arrival.setSize(QUEUE_MAX_SIZE+1);

    vq.update(QUEUE_MAX_SIZE + 1, 0);
    EXPECT_EQ(vq.getSize(), QUEUE_MAX_SIZE);
    vq.update(0, 2*QUEUE_MAX_SIZE+1);
    EXPECT_EQ(vq.getSize(), -QUEUE_MAX_SIZE);

    vq.update(vq_arrival, 0);
    vq.update(vq_arrival, 0);
    vq.update(vq_arrival, 0);
    EXPECT_EQ(vq.getSize(), QUEUE_MAX_SIZE);

    vq.update(2,3);
    EXPECT_EQ(vq.getSize(), 9);
}

TEST(EqConVirtualQueueDynamicTest, predictionTest)
{
    const int QUEUE_MAX_SIZE = 10;
    EqConVirtualQueueEvaluation vq(QUEUE_MAX_SIZE);
    vq.predicted_arrival_ = 2;
    vq.predicted_transmission_ = 3;

    vq.update(4,0);
    EXPECT_EQ(vq.evaluate(), 4 - vq.predicted_transmission_ + vq.predicted_arrival_);
    
    // Evaluation has no effect on real queue size
    EXPECT_EQ(vq.getSize(), 4);

    // Max bound evaluation limit
    vq.predicted_arrival_ = 7;
    vq.predicted_transmission_ = 0;
    EXPECT_EQ(vq.evaluate(), QUEUE_MAX_SIZE);

    // Min bound evaluation limit
    vq.predicted_arrival_ = 0;
    vq.predicted_transmission_ = 20;
    EXPECT_EQ(vq.evaluate(), -QUEUE_MAX_SIZE);

    // Ilegal prediction
    vq.predicted_arrival_ = -1;
    EXPECT_THROW(vq.evaluate(), NegativeArrivalPredictionException);
    
    vq.predicted_arrival_ = 0;
    vq.predicted_transmission_ = -1;
    EXPECT_THROW(vq.evaluate(), NegativeDeparturePredictionException);
}

TEST(EqConVirtualQueueDynamicTest, memorySizeTest)
{
    const int QUEUE_MAX_SIZE = 10;
    EqConVirtualQueue vq(QUEUE_MAX_SIZE);

    EXPECT_EQ(vq.getMemSize(), sizeof(int));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}