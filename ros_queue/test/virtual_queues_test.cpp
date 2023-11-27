#include <stdexcept>

#include <gtest/gtest.h>

#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queue/lib_queue/virtual_queue.hpp"

#include "ros_queue/lib_queue/queue_exception.hpp"


using std::invalid_argument;


TEST(VirtualQueueTest, manipulationTest)
{
    VirtualQueue vq;

    EXPECT_FLOAT_EQ(vq.size(), 0);
    vq.push(2.0f);
    EXPECT_FLOAT_EQ(vq.size(), 2.0f);
    vq.push(2.0f);
    EXPECT_FLOAT_EQ(vq.size(), 4.0f);

    vq.pop(2.0f);
    EXPECT_FLOAT_EQ(vq.size(), 2.0f);
    vq.pop(2.0f);
    EXPECT_FLOAT_EQ(vq.size(), 0.0f);

    vq.setSize(5.0f);
    EXPECT_FLOAT_EQ(vq.size(), 5.0f);

    EXPECT_THROW(vq.push(-7.0f),invalid_argument);
    EXPECT_THROW(vq.pop(-7.0f),invalid_argument);

    EXPECT_THROW(vq.setSize(-1.0f), invalid_argument);
}

TEST(VirtualQueueTest, dimensionLimitTest)
{
    VirtualQueue vq;

    EXPECT_FLOAT_EQ(vq.empty(), true);
    vq.push(2.0f);
    EXPECT_FLOAT_EQ(vq.empty(), false);

    vq.pop(4.0f);
    EXPECT_FLOAT_EQ(vq.size(), 0.0f);
    vq.pop(2.0f);
    EXPECT_FLOAT_EQ(vq.size(), 0.0f);
}

TEST(NVirtualQueueTest, manipulationTest)
{
    NVirtualQueue vq;

    EXPECT_FLOAT_EQ(vq.size(), 0.0f);
    vq.push(2.0f);
    EXPECT_FLOAT_EQ(vq.size(), 2.0f);
    vq.push(2.0f);
    EXPECT_FLOAT_EQ(vq.size(), 4.0f);

    vq.pop(2.0f);
    EXPECT_FLOAT_EQ(vq.size(), 2.0f);
    vq.pop(2.0f);
    EXPECT_FLOAT_EQ(vq.size(), 0.0f);
    vq.pop(3.0f);
    EXPECT_FLOAT_EQ(vq.size(), -3.0f);

    vq.setSize(5.0f);
    EXPECT_FLOAT_EQ(vq.size(), 5.0f);

    EXPECT_THROW(vq.push(-7.0f),invalid_argument);
    EXPECT_THROW(vq.pop(-7.0f),invalid_argument);
    
    vq.setSize(-1.0f);
    EXPECT_FLOAT_EQ(vq.size(), -1.0f);
}

TEST(NVirtualQueueTest, dimensionLimitTest)
{
    NVirtualQueue vq;

    EXPECT_FLOAT_EQ(vq.empty(), true);
    vq.push(2.0f);
    EXPECT_FLOAT_EQ(vq.empty(), false);
    vq.setSize(-2.0f);
    EXPECT_FLOAT_EQ(vq.empty(), false);
}

// Test for the inequqality constraint virtual queue
TEST(InConVirutalQueueDynamicTest, manipultationTest)
{
    InConVirtualQueue vq(10.0f);

    EXPECT_FLOAT_EQ(vq.getSize(), 0);
    vq.update(4.0f-3.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), 1.0f);
    vq.update(4.0f-3.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), 2.0f);
}

TEST(InConVirutalQueueDynamicTest, initTest)
{
    const int QUEUE_MAX_SIZE = -10.0f;
    EXPECT_THROW(InConVirtualQueue vq(QUEUE_MAX_SIZE), invalid_argument);
}

TEST(InConVirutalQueueDynamicTest, dimensionLimitTest)
{
    const int QUEUE_MAX_SIZE = 10.0f;
    InConVirtualQueue vq(QUEUE_MAX_SIZE);

    vq.update(-1.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), 0.0f);
    vq.update(QUEUE_MAX_SIZE + 1.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), QUEUE_MAX_SIZE);

    vq.update(QUEUE_MAX_SIZE + 1.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), QUEUE_MAX_SIZE);
}

// Test for the equality constraint virtual queue
TEST(EqConVirtualQueueDynamicTest, manipultationTest)
{
    EqConVirtualQueue vq(10.0f);

    EXPECT_FLOAT_EQ(vq.getSize(), 0.0f);
    vq.update(4.0f-3.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), 1.0f);
    vq.update(4.0f-3.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), 2.0f);

    vq.update(-9.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), -7.0f);
}

TEST(EqConVirtualQueueDynamicTest, initTest)
{
    const int QUEUE_MAX_SIZE = -10.0f;
    EXPECT_THROW(EqConVirtualQueue vq(QUEUE_MAX_SIZE), invalid_argument);
}

TEST(EqConVirtualQueueDynamicTest, dimensionLimitTest)
{
    const int QUEUE_MAX_SIZE = 10.0f;
    EqConVirtualQueue vq(QUEUE_MAX_SIZE);

    vq.update(QUEUE_MAX_SIZE + 1.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), QUEUE_MAX_SIZE);
    vq.update(-2*QUEUE_MAX_SIZE-1.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), -QUEUE_MAX_SIZE);

    vq.update(1.0f);
    EXPECT_FLOAT_EQ(vq.getSize(), -9.0f);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}