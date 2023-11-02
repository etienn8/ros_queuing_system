#include <string>
#include <deque>

#include <gtest/gtest.h>

#include "ros/ros.h"

#include "ros_queue/ReturnSentValue.h"

#include "ros_queue/ros_virtual_queue.hpp"
#include "ros_queue/ros_queue.hpp"
#include "ros_queue/ros_queue_info.hpp"

#include "include/trajectory.hpp"

#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queue/lib_queue/dynamic_queue.hpp"
#include "ros_queue/lib_queue/dynamic_converted_queue.hpp"
#include "ros_queue/lib_queue/queue_utils.hpp"

using namespace std;

static const list<Position3D> point_list_0 = {Position3D(1,1,1), Position3D(2,2,2), Position3D(3,3,3)};
static const list<Position3D> point_list_1 = {Position3D(4,4,4), Position3D(5,5,5), Position3D(6,6,6), Position3D(7,7,7)};

class RosVirtualQueueFixture : public testing::Test {
    protected:
        void SetUp() override {
            arrival_prediction_service_name_f= "/return_sent_value_plus_two";
            transmission_prediction_service_name_f = "/return_sent_value_plus_three";
            max_queue_size_f = 10;
            service_struct_test_f.request.value_to_return = 1;
        }
        
        int max_queue_size_f;
        string arrival_prediction_service_name_f;
        string transmission_prediction_service_name_f;
        ros::NodeHandle nh_f;

        ros_queue::ReturnSentValue service_struct_test_f;

        ROSQueueInfo queue_info_f;
};

class RosQueueFixture : public testing::Test {
    public:
        typedef int elementQueueType;

    protected:
        void SetUp() override {
            arrival_prediction_service_name_f= "/return_sent_value_plus_two";
            transmission_prediction_service_name_f = "/return_sent_value_plus_three";
            transmission_topic_name_f = "/queue_transmission";
            max_queue_size_f = 10;
            service_struct_test_f.request.value_to_return = 1;
        }

        int max_queue_size_f;
        string arrival_prediction_service_name_f;
        string transmission_prediction_service_name_f;
        string transmission_topic_name_f;

        ros::NodeHandle nh_f;

        deque<elementQueueType> arrival_queue_f = {1, 2, 3, 4};

        
        ros_queue::ReturnSentValue service_struct_test_f;

        ROSQueueInfo queue_info_f;
};

class RosConvertedQueueFixture : public testing::Test {
    public:
        typedef Trajectory elementQueueType;

    protected:
        void SetUp() override {
            arrival_prediction_service_name_f= "/return_sent_value_plus_two";
            transmission_prediction_service_name_f = "/return_sent_value_plus_three";
            max_queue_size_f = 512;
            service_struct_test_f.request.value_to_return = 1;
        }

        int max_queue_size_f;
        string arrival_prediction_service_name_f;
        string transmission_prediction_service_name_f;
        ros::NodeHandle nh_f;

        deque<Trajectory> arrival_queue_f = {Trajectory("frame_0", point_list_0), Trajectory("frame_1", point_list_1), Trajectory("frame_2", point_list_0)};


        ros_queue::ReturnSentValue service_struct_test_f;

        ROSQueueInfo queue_info_f;
};


namespace prediction
{
    int return_value_plus_one(const ros_queue::ReturnSentValue& service)
    {
        return service.request.value_to_return+1;
    }

    int return_value(const ros_queue::ReturnSentValue& service)
    {
        return service.request.value_to_return;
    }
}

namespace transmission
{
    deque<int> output_int_dequeue_1;
    deque<int> output_int_dequeue_2;

    bool transmission_on_dequeue1(deque<int> &queue_to_transmit)
    {
        queue_utils::concatenate_queues(output_int_dequeue_1, queue_to_transmit);
        return true;
    }

    bool transmission_on_dequeue2(deque<int> &queue_to_transmit)
    {
        queue_utils::concatenate_queues(output_int_dequeue_2, queue_to_transmit);
        return true;
    }

}


TEST_F(RosVirtualQueueFixture, constructorOverrideTest)
{
    // InConVirtualQueue queue with user-defined transmission and user-defined arrival
    ROSVirtualQueue<InConVirtualQueue, ros_queue::ReturnSentValue> vq0(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value);
    vq0.update(4,0);
    EXPECT_EQ(vq0.evaluate(service_struct_test_f), 5);

    // InConVirtualQueue queue with user-defined transmission and ROS service arrival
    ROSVirtualQueue<InConVirtualQueue, ros_queue::ReturnSentValue> vq1(max_queue_size_f, queue_info_f, nh_f, prediction::return_value_plus_one, transmission_prediction_service_name_f);
    vq1.update(4,0);
    EXPECT_EQ(vq1.evaluate(service_struct_test_f), 2);

    // InConVirtualQueue queue with ROS service transmission and user-defined arrival
    ROSVirtualQueue<InConVirtualQueue, ros_queue::ReturnSentValue>vq2(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, prediction::return_value);
    vq2.update(4,0);
    EXPECT_EQ(vq2.evaluate(service_struct_test_f), 6);

    // InConVirtualQueue queue with ROS service transmission and ROS servicearrival
    ROSVirtualQueue<InConVirtualQueue, ros_queue::ReturnSentValue> vq3(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, transmission_prediction_service_name_f);
    vq3.update(4,0);
    EXPECT_EQ(vq3.evaluate(service_struct_test_f), 3);


    // EqConVirtualQueue queue with user-defined transmission and user-defined arrival
    ROSVirtualQueue<EqConVirtualQueue, ros_queue::ReturnSentValue> vq4(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value);
    vq4.update(4,0);
    EXPECT_EQ(vq4.evaluate(service_struct_test_f), 5);
    
    // EqConVirtualQueue queue with user-defined transmission and ROS service arrival
    ROSVirtualQueue<EqConVirtualQueue, ros_queue::ReturnSentValue> vq5(max_queue_size_f, queue_info_f, nh_f, prediction::return_value_plus_one, transmission_prediction_service_name_f);
    vq5.update(4,0);
    EXPECT_EQ(vq5.evaluate(service_struct_test_f), 2);
    
    // EqConVirtualQueue queue with ROS service transmission and user-defined arrival
    ROSVirtualQueue<EqConVirtualQueue, ros_queue::ReturnSentValue> vq6(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, prediction::return_value);
    vq6.update(4,0);
    EXPECT_EQ(vq6.evaluate(service_struct_test_f), 6);
    
    // EqConVirtualQueue queue with ROS service transmission and ROS servicearrival
    ROSVirtualQueue<EqConVirtualQueue, ros_queue::ReturnSentValue> vq7(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, transmission_prediction_service_name_f);
    vq7.update(4,0);
    EXPECT_EQ(vq7.evaluate(service_struct_test_f), 3);
}

TEST_F(RosVirtualQueueFixture, badInitTest)
{
    //Test all constructors with a bad initialization of their function pointers.

    // Create a type since the EXPECT_THROW macros confuses the coma in the template's parameters as a separator for its macro arguments.
    typedef ROSVirtualQueue<InConVirtualQueue,ros_queue::ReturnSentValue> ROSInConVirtualQUeue; 

    // InConVirtualQueue queue with user-defined transmission and user-defined arrival
    EXPECT_THROW(ROSInConVirtualQUeue vq0(max_queue_size_f, queue_info_f, nullptr, prediction::return_value), invalid_argument);

    // InConVirtualQueue queue with user-defined transmission and user-defined arrival
    EXPECT_THROW(ROSInConVirtualQUeue vq0(max_queue_size_f, queue_info_f, prediction::return_value, nullptr), invalid_argument);

    // InConVirtualQueue queue with user-defined transmission and ROS service arrival
    EXPECT_THROW(ROSInConVirtualQUeue vq1(max_queue_size_f, queue_info_f, nh_f, nullptr, transmission_prediction_service_name_f), invalid_argument);

    // InConVirtualQueue queue with ROS service transmission and user-defined arrival
    EXPECT_THROW(ROSInConVirtualQUeue vq2(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, nullptr), invalid_argument);

    typedef ROSVirtualQueue<EqConVirtualQueue,ros_queue::ReturnSentValue> ROSEqConVirtualQUeue; 
    // InConVirtualQueue queue with user-defined transmission and user-defined arrival
    EXPECT_THROW(ROSEqConVirtualQUeue vq3(max_queue_size_f, queue_info_f, nullptr, prediction::return_value), invalid_argument);

    // InConVirtualQueue queue with user-defined transmission and user-defined arrival
    EXPECT_THROW(ROSEqConVirtualQUeue vq4(max_queue_size_f, queue_info_f, prediction::return_value, nullptr), invalid_argument);

    // InConVirtualQueue queue with user-defined transmission and ROS service arrival
    EXPECT_THROW(ROSEqConVirtualQUeue vq5(max_queue_size_f, queue_info_f, nh_f, nullptr, transmission_prediction_service_name_f), invalid_argument);

    // InConVirtualQueue queue with ROS service transmission and user-defined arrival
    EXPECT_THROW(ROSEqConVirtualQUeue vq6(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, nullptr), invalid_argument);

}


TEST_F(RosQueueFixture, constructorOverrideTest)
{
    // ROSQueue queue with user-defined transmission, user-defined arrival and user-defined transmission
    ROSQueue<int, ros_queue::ReturnSentValue> vq0(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value, transmission::transmission_on_dequeue1);
    vq0.update(arrival_queue_f,0);
    EXPECT_EQ(vq0.evaluate(service_struct_test_f), 5);

    // ROSQueue queue with user-defined transmission, ROS service arrival and user-defined transmission
    ROSQueue<int, ros_queue::ReturnSentValue> vq1(max_queue_size_f, queue_info_f, nh_f, prediction::return_value_plus_one, transmission_prediction_service_name_f, transmission::transmission_on_dequeue1);
    vq1.update(arrival_queue_f,0);
    EXPECT_EQ(vq1.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival and user-defined transmission
    ROSQueue<int, ros_queue::ReturnSentValue> vq2(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, prediction::return_value, transmission::transmission_on_dequeue1);
    vq2.update(arrival_queue_f,0);
    EXPECT_EQ(vq2.evaluate(service_struct_test_f), 6);

    // ROSQueue queue with ROS service transmission, ROS servicearrival and user-defined transmission
    ROSQueue<int, ros_queue::ReturnSentValue>  vq3(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, transmission_prediction_service_name_f, transmission::transmission_on_dequeue1);
    vq3.update(arrival_queue_f,0);
    EXPECT_EQ(vq3.evaluate(service_struct_test_f), 3);

    // ROSQueue queue with user-defined transmission, user-defined arrival and transmission topic
    ROSQueue<int, ros_queue::ReturnSentValue> vq4(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value, transmission_topic_name_f);
    vq4.update(arrival_queue_f,0);
    EXPECT_EQ(vq4.evaluate(service_struct_test_f), 5);

    // ROSQueue queue with user-defined transmission, ROS service arrival and transmission topic
    ROSQueue<int, ros_queue::ReturnSentValue> vq5(max_queue_size_f, queue_info_f, nh_f, prediction::return_value_plus_one, transmission_prediction_service_name_f, transmission_topic_name_f);
    vq5.update(arrival_queue_f,0);
    EXPECT_EQ(vq5.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival and transmission topic
    ROSQueue<int, ros_queue::ReturnSentValue> vq6(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, prediction::return_value, transmission_topic_name_f);
    vq6.update(arrival_queue_f,0);
    EXPECT_EQ(vq6.evaluate(service_struct_test_f), 6);

    // ROSQueue queue with ROS service transmission, ROS servicearrival and transmission topic
    ROSQueue<int, ros_queue::ReturnSentValue>  vq7(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, transmission_prediction_service_name_f, transmission_topic_name_f);
    vq7.update(arrival_queue_f,0);
    EXPECT_EQ(vq7.evaluate(service_struct_test_f), 3);
}

TEST_F(RosQueueFixture, badInitTest)
{
    //Test all constructors with a bad initialization of their function pointers.

    // Create a type since the EXPECT_THROW macros confuses the coma in the template's parameters as a separator for its macro arguments.
    typedef ROSQueue<int,ros_queue::ReturnSentValue> ROSIntQueue; 

    EXPECT_THROW(ROSIntQueue vq0(max_queue_size_f, queue_info_f, nullptr, prediction::return_value, transmission::transmission_on_dequeue1), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq1(max_queue_size_f, queue_info_f, prediction::return_value, nullptr, transmission::transmission_on_dequeue1), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq2(max_queue_size_f, queue_info_f, nh_f, nullptr, transmission_prediction_service_name_f, transmission::transmission_on_dequeue1), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq3(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, nullptr, transmission::transmission_on_dequeue1), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq4(max_queue_size_f, queue_info_f, nullptr, prediction::return_value, transmission_topic_name_f), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq5(max_queue_size_f, queue_info_f, prediction::return_value, nullptr, transmission_topic_name_f), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq6(max_queue_size_f, queue_info_f, nh_f, nullptr, transmission_prediction_service_name_f, transmission_topic_name_f), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq7(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, nullptr, transmission_topic_name_f), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq8(max_queue_size_f, queue_info_f, nullptr, prediction::return_value, nullptr), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq9(max_queue_size_f, queue_info_f, prediction::return_value, nullptr, nullptr), invalid_argument);

    EXPECT_THROW(ROSIntQueue vq10(max_queue_size_f, queue_info_f, nh_f, nullptr, transmission_prediction_service_name_f, nullptr), invalid_argument);

    EXPECT_THROW(ROSIntQueue v11(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, nullptr, nullptr), invalid_argument);
}

TEST_F(RosQueueFixture, transmissionTest)
{
    // Clear the test transmission queue is empty
    transmission::output_int_dequeue_1.clear();
    EXPECT_EQ(transmission::output_int_dequeue_1.size(), 0);

    // Test if the user-defined transmission works
    ROSQueue<int, ros_queue::ReturnSentValue> vq0(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value, transmission::transmission_on_dequeue1);
    vq0.update(arrival_queue_f,0);
    EXPECT_EQ(transmission::output_int_dequeue_1.size(), 0);
    vq0.update(arrival_queue_f,2);
    deque<int> compared_queue = {1,2};
    EXPECT_EQ(transmission::output_int_dequeue_1, compared_queue);
    
    // Test if the transmission through ROS Topics works
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ros_queue_test");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}