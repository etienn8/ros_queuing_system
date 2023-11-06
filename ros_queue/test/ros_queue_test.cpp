#include "include/ros_queue_test.hpp"
#include <string>
#include <deque>

#include <gtest/gtest.h>

#include "ros/ros.h"

// ROS Services and messages
#include "ros_queue/ReturnSentValue.h"
#include "ros_queue/queue_transmit_template.h"
#include "ros_queue/queue_int_element.h"

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
        typedef ROSQueue<ros_queue::queue_transmit_template, ros_queue::ReturnSentValue> ROSIntQueue_f; 

    protected:
        void SetUp() override {
            arrival_prediction_service_name_f= "/return_sent_value_plus_two";
            transmission_prediction_service_name_f = "/return_sent_value_plus_three";
            transmission_topic_name_f = QUEUE_TOPIC_TRANSMISSION_NAME;
            max_queue_size_f = 10;
            service_struct_test_f.request.value_to_return = 1;

            ros_queue::queue_int_element element_to_push;
            for (int i=1; i <= 4; ++i)
            {
                element_to_push.value = i;
                arrival_queue_f.push_back(element_to_push);
            }
            
        }

        int max_queue_size_f;
        string arrival_prediction_service_name_f;
        string transmission_prediction_service_name_f;
        string transmission_topic_name_f;

        ros::NodeHandle nh_f;

        deque<ros_queue::queue_int_element> arrival_queue_f;


        
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
    deque<ros_queue::queue_int_element> output_int_dequeue_1;
    deque<ros_queue::queue_int_element> output_int_dequeue_2;

    bool transmission_on_dequeue1(deque<ros_queue::queue_int_element> &queue_to_transmit)
    {
        queue_utils::concatenate_queues(transmission::output_int_dequeue_1, queue_to_transmit);
        return true;
    }

    bool transmission_on_dequeue2(deque<ros_queue::queue_int_element> &queue_to_transmit)
    {
        queue_utils::concatenate_queues(transmission::output_int_dequeue_2, queue_to_transmit);
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
    RosQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value, transmission::transmission_on_dequeue1);
    vq0.update(arrival_queue_f,0);
    EXPECT_EQ(vq0.evaluate(service_struct_test_f), 5);

    // ROSQueue queue with user-defined transmission, ROS service arrival and user-defined transmission
    RosQueueFixture::ROSIntQueue_f vq1(max_queue_size_f, queue_info_f, nh_f, prediction::return_value_plus_one, transmission_prediction_service_name_f, transmission::transmission_on_dequeue1);
    vq1.update(arrival_queue_f,0);
    EXPECT_EQ(vq1.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival and user-defined transmission
    RosQueueFixture::ROSIntQueue_f vq2(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, prediction::return_value, transmission::transmission_on_dequeue1);
    vq2.update(arrival_queue_f,0);
    EXPECT_EQ(vq2.evaluate(service_struct_test_f), 6);

    // ROSQueue queue with ROS service transmission, ROS servicearrival and user-defined transmission
    RosQueueFixture::ROSIntQueue_f  vq3(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, transmission_prediction_service_name_f, transmission::transmission_on_dequeue1);
    vq3.update(arrival_queue_f,0);
    EXPECT_EQ(vq3.evaluate(service_struct_test_f), 3);

    // ROSQueue queue with user-defined transmission, user-defined arrival and transmission topic
    RosQueueFixture::ROSIntQueue_f vq4(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value, transmission_topic_name_f);
    vq4.update(arrival_queue_f,0);
    EXPECT_EQ(vq4.evaluate(service_struct_test_f), 5);

    // ROSQueue queue with user-defined transmission, ROS service arrival and transmission topic
    RosQueueFixture::ROSIntQueue_f vq5(max_queue_size_f, queue_info_f, nh_f, prediction::return_value_plus_one, transmission_prediction_service_name_f, transmission_topic_name_f);
    vq5.update(arrival_queue_f,0);
    EXPECT_EQ(vq5.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival and transmission topic
    RosQueueFixture::ROSIntQueue_f vq6(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, prediction::return_value, transmission_topic_name_f);
    vq6.update(arrival_queue_f,0);
    EXPECT_EQ(vq6.evaluate(service_struct_test_f), 6);

    // ROSQueue queue with ROS service transmission, ROS servicearrival and transmission topic
    RosQueueFixture::ROSIntQueue_f vq7(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, transmission_prediction_service_name_f, transmission_topic_name_f);
    vq7.update(arrival_queue_f,0);
    EXPECT_EQ(vq7.evaluate(service_struct_test_f), 3);
}

TEST_F(RosQueueFixture, badInitTest)
{
    //Test all constructors with a bad initialization of their function pointers

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nullptr, prediction::return_value, transmission::transmission_on_dequeue1), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq1(max_queue_size_f, queue_info_f, prediction::return_value, nullptr, transmission::transmission_on_dequeue1), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq2(max_queue_size_f, queue_info_f, nh_f, nullptr, transmission_prediction_service_name_f, transmission::transmission_on_dequeue1), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq3(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, nullptr, transmission::transmission_on_dequeue1), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq4(max_queue_size_f, queue_info_f, nullptr, prediction::return_value, transmission_topic_name_f), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq5(max_queue_size_f, queue_info_f, prediction::return_value, nullptr, transmission_topic_name_f), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq6(max_queue_size_f, queue_info_f, nh_f, nullptr, transmission_prediction_service_name_f, transmission_topic_name_f), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq7(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, nullptr, transmission_topic_name_f), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq8(max_queue_size_f, queue_info_f, nullptr, prediction::return_value, nullptr), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq9(max_queue_size_f, queue_info_f, prediction::return_value, nullptr, nullptr), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq10(max_queue_size_f, queue_info_f, nh_f, nullptr, transmission_prediction_service_name_f, nullptr), invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f v11(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, nullptr, nullptr), invalid_argument);
}

TEST_F(RosQueueFixture, transmissionTest)
{
    // Clear the test transmission queue is empty
    transmission::output_int_dequeue_1.clear();
    EXPECT_EQ(transmission::output_int_dequeue_1.size(), 0);

    // Test if the user-defined transmission works
    RosQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value, transmission::transmission_on_dequeue1);
    vq0.update(arrival_queue_f,0);
    EXPECT_EQ(transmission::output_int_dequeue_1.size(), 0);
    vq0.update(arrival_queue_f, 2);
    EXPECT_EQ(vq0.getSize(), 6);

    
    deque<ros_queue::queue_int_element> compared_queue;
    ros_queue::queue_int_element int_element;
    int_element.value =1;
    compared_queue.push_back(int_element);
    int_element.value = 2;
    compared_queue.push_back(int_element);

    EXPECT_EQ(transmission::output_int_dequeue_1, compared_queue);
    
    // Test if the transmission through ROS Topics works. EDIT: Verified with in another way.

    /* RosQueueFixture::ROSIntQueue_f vq1(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value, transmission_topic_name_f);
    vq1.update(arrival_queue_f,0);
    vq1.update(arrival_queue_f, 2);*/

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ros_queue_test");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}