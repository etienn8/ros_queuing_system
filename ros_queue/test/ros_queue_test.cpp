#include <string>

#include <gtest/gtest.h>

#include "ros/ros.h"

#include "ros_queue/ReturnSentValue.h"

#include "ros_queue/ros_queue.hpp"
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queue/ros_queue_info.hpp"

using namespace std;

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


TEST_F(RosVirtualQueueFixture, constructorOverrideTest)
{

    // InConVirtualQueue queue with user-defined transmission and user-defined arrival
    ROSQueue<InConVirtualQueue<ros_queue::ReturnSentValue>,ros_queue::ReturnSentValue> vq0(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value);
    vq0.update(4,0);
    EXPECT_EQ(vq0.evaluate(service_struct_test_f), 5);

    // InConVirtualQueue queue with user-defined transmission and ROS service arrival
    ROSQueue<InConVirtualQueue<ros_queue::ReturnSentValue>,ros_queue::ReturnSentValue> vq1(max_queue_size_f, queue_info_f, nh_f, prediction::return_value_plus_one, transmission_prediction_service_name_f);
    vq1.update(4,0);
    EXPECT_EQ(vq1.evaluate(service_struct_test_f), 2);

    // InConVirtualQueue queue with ROS service transmission and user-defined arrival
    ROSQueue<InConVirtualQueue<ros_queue::ReturnSentValue>, ros_queue::ReturnSentValue> vq2(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, prediction::return_value);
    vq2.update(4,0);
    EXPECT_EQ(vq2.evaluate(service_struct_test_f), 6);

    // InConVirtualQueue queue with ROS service transmission and ROS servicearrival
    ROSQueue<InConVirtualQueue<ros_queue::ReturnSentValue>, ros_queue::ReturnSentValue> vq3(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, transmission_prediction_service_name_f);
    vq3.update(4,0);
    EXPECT_EQ(vq3.evaluate(service_struct_test_f), 3);


    // EqConVirtualQueue queue with user-defined transmission and user-defined arrival
    ROSQueue<EqConVirtualQueue<ros_queue::ReturnSentValue>, ros_queue::ReturnSentValue> vq4(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value);
    vq4.update(4,0);
    EXPECT_EQ(vq4.evaluate(service_struct_test_f), 5);
    
    // EqConVirtualQueue queue with user-defined transmission and ROS service arrival
    ROSQueue<EqConVirtualQueue<ros_queue::ReturnSentValue>, ros_queue::ReturnSentValue> vq5(max_queue_size_f, queue_info_f, nh_f, prediction::return_value_plus_one, transmission_prediction_service_name_f);
    vq5.update(4,0);
    EXPECT_EQ(vq5.evaluate(service_struct_test_f), 2);
    
    // EqConVirtualQueue queue with ROS service transmission and user-defined arrival
    ROSQueue<EqConVirtualQueue<ros_queue::ReturnSentValue>, ros_queue::ReturnSentValue> vq6(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, prediction::return_value);
    vq6.update(4,0);
    EXPECT_EQ(vq6.evaluate(service_struct_test_f), 6);
    
    // EqConVirtualQueue queue with ROS service transmission and ROS servicearrival
    ROSQueue<EqConVirtualQueue<ros_queue::ReturnSentValue>, ros_queue::ReturnSentValue> vq7(max_queue_size_f, queue_info_f, nh_f, arrival_prediction_service_name_f, transmission_prediction_service_name_f);
    vq7.update(4,0);
    EXPECT_EQ(vq7.evaluate(service_struct_test_f), 3);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ros_queue_test");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}