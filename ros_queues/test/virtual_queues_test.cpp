#include <deque>

#include <gtest/gtest.h>

#include "ros_queues/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queues/lib_queue/dynamic_queue.hpp"


TEST(InConVirutalQueueDynamicTest, dimensionTests)
{
    InConVirtualQueue vq1(100);

    EXPECT_EQ(vq1.getSize(), 0);
    vq1.udpate(4,3);
    EXPECT_EQ(vq1.getSize(), 4);
    vq1.udpate(4,3);
    EXPECT_EQ(vq1.getSize(), 5);
    vq1.udpate(4,3);
    EXPECT_EQ(vq1.getSize(), 6);
    vq1.udpate(0,20);
    EXPECT_EQ(vq1.getSize(), 0);
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}