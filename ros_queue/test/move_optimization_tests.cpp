#include <gtest/gtest.h>

#include <vector>
#include <utility>
#include <iostream>

#include "ros_queue/ros_converted_queue.hpp"
#include "ros_queue/ros_queue.hpp"

#include "ros_queue/lib_queue/dynamic_queue.hpp"
#include "ros_queue/lib_queue/dynamic_converted_queue.hpp"

#include "include/dynamic_queues_test.hpp"

#include "ros/ros.h"

using namespace std;

class Pos
{
    public:
        int x_;
        int y_;
        int z_;

        Pos(int x, int y, int z): x_(x), y_(y), z_(z) {}
};

class Traj
{
    public:
        vector<Pos> point_list;
        static int copy_count;
        static int move_count;
        static bool count_started;

        static void resetAndStartCount()
        {
            Traj::count_started = true;
            Traj::copy_count = 0;
            Traj::move_count = 0;
        }

        Traj() {};
        ~Traj() {};

        Traj(const Traj& copy_traj)
        {
            if (count_started)
            {
                ++Traj::copy_count;
            }
            this->point_list = copy_traj.point_list;
        }

        Traj(Traj&& move_traj)
        {
            if (count_started)
            {
                ++Traj::move_count;
            }
            this->point_list = std::move(move_traj.point_list);
        }

        // user-defined copy assignment (copy-and-swap idiom)
        Traj& operator=(Traj other)
        {
            if (count_started)
            {
                ++Traj::copy_count;
            }
            std::swap(point_list, other.point_list);
            return *this;
        }
};

// Initialization static variables
bool Traj::count_started = false;
int Traj::copy_count = 0;
int Traj::move_count = 0;

class MsgMock
{
    public:
        vector<Traj> queue_elements;
};



void traj_to_byte_conversion(deque<Traj>&& queue_trajectory, deque<ElementWithConvertedSize<Traj>>& converted_queue)
{
    for(typename deque<Traj>::iterator it = queue_trajectory.begin(); it != queue_trajectory.end(); ++it)
    {
        int converted_size = sizeof(Traj);

        ElementWithConvertedSize<Traj> convertedElement(std::move(*it), converted_size);
        converted_queue.push_back(std::move(convertedElement));
    }
}


TEST(LibQueueNoCopyTestSuite, dynamicQueue)
{
    DynamicQueue<Traj> q(10);
    deque<Traj> empty_queue;

    // Initialize queue to inject
    deque<Traj> arrival_queue;
    for (int index_traj=0; index_traj <3; ++index_traj)
    {
        Traj new_traj_to_add;
        for (int index_pos=0; index_pos <4; ++index_pos)
        {
            new_traj_to_add.point_list.push_back(Pos(index_pos+index_traj*10, index_pos+index_traj*10, index_pos+index_traj*10));
        }
        arrival_queue.push_back(new_traj_to_add);
    }

    Traj::resetAndStartCount();
    EXPECT_EQ(Traj::copy_count, 0);
    EXPECT_EQ(Traj::move_count, 0);

    const deque<Traj> empty_deque;
    // Add some elements and verify expected size and content
    q.update(arrival_queue, 0);

    EXPECT_EQ(Traj::copy_count, 3);
    EXPECT_EQ(Traj::move_count, 3);

    Traj::resetAndStartCount();
    q.update(std::move(arrival_queue), 0);

    EXPECT_EQ(Traj::copy_count, 0);
    EXPECT_EQ(Traj::move_count, 3);

    Traj::resetAndStartCount();
    q.update(empty_queue, 2);

    EXPECT_EQ(Traj::copy_count, 0);
    EXPECT_EQ(Traj::move_count, 2);

}

TEST(LibQueueNoCopyTestSuite, convertedQueueTest)
{
    const int queue_max_size = 512;
    DynamicConvertedQueueWithFPtr<Traj> q(queue_max_size, traj_to_byte_conversion);

    deque<Traj> empty_queue;

    // Initialize queue to inject
    deque<Traj> arrival_queue;
    for (int index_traj=0; index_traj <3; ++index_traj)
    {
        Traj new_traj_to_add;
        for (int index_pos=0; index_pos <4; ++index_pos)
        {
            new_traj_to_add.point_list.push_back(Pos(index_pos+index_traj*10, index_pos+index_traj*10, index_pos+index_traj*10));
        }
        arrival_queue.push_back(new_traj_to_add);
    }

    Traj::resetAndStartCount();
    EXPECT_EQ(Traj::copy_count, 0);
    EXPECT_EQ(Traj::move_count, 0);

    // Add some elements and verify expected size and content
    q.update(arrival_queue, 0);

    EXPECT_EQ(Traj::copy_count, 3);
    EXPECT_EQ(Traj::move_count, 9);

    Traj::resetAndStartCount();
    q.update(std::move(arrival_queue), 0);

    EXPECT_EQ(Traj::copy_count, 0);
    EXPECT_EQ(Traj::move_count, 9);

    Traj::resetAndStartCount();
    q.update(empty_queue, 2);

    EXPECT_EQ(Traj::copy_count, 0);
    EXPECT_EQ(Traj::move_count, 2);

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}