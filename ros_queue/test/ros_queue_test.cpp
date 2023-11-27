#include "include/ros_queue_test.hpp"
#include <string>
#include <deque>

#include <gtest/gtest.h>

#include "ros/ros.h"

// ROS Services and messages
#include "ros_queue/ReturnSentValue.h"
#include "ros_queue/queue_transmit_template.h"
#include "ros_queue/queue_int_element.h"
#include "ros_queue/ConversionTemplateService.h"

#include "ros_queue/ros_converted_queue.hpp"
#include "ros_queue/ros_queue.hpp"
#include "ros_queue/ros_queue_info.hpp"
#include "ros_queue/ros_virtual_queue.hpp"

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
            metric_computation_service_name_f= "/return_sent_value_plus_1point2";
            max_queue_size_f = 10;
            target_metric_f = 3.0f;
        }
        
        int max_queue_size_f;
        string metric_computation_service_name_f;

        ros::NodeHandle nh_f;

        ros_queue::ReturnSentValue service_struct_test_f;

        ROSQueueInfo queue_info_f;

        float target_metric_f; 
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
        typedef ROSConvertedQueue<ros_queue::queue_transmit_template, ros_queue::ReturnSentValue, ros_queue::ConversionTemplateService> ROSIntQueue_f; 

    protected:
        void SetUp() override {
            arrival_prediction_service_name_f= "/return_sent_value_plus_two";
            transmission_prediction_service_name_f = "/return_sent_value_plus_three";
            conversion_service_name_f = "/conversion_to_bytes_service";
            transmission_topic_name_f = QUEUE_TOPIC_TRANSMISSION_NAME;
            max_queue_size_f = 512;
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
        string conversion_service_name_f;

        ros::NodeHandle nh_f;

        deque<ros_queue::queue_int_element> arrival_queue_f;
        
        ros_queue::ReturnSentValue service_struct_test_f;

        ROSQueueInfo queue_info_f;
};

namespace metric_computation
{
    float mocked_metric;

    float getMockedMetric()
    {
        return mocked_metric;
    }
}

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

    bool transmission_on_dequeue1(deque<ros_queue::queue_int_element>&& queue_to_transmit)
    {
        queue_utils::concatenate_queues(transmission::output_int_dequeue_1, std::move(queue_to_transmit));
        return true;
    }
}

namespace conversion
{
    void conversion_int_to_byte(deque<ros_queue::queue_int_element>&& arriving_queue,  deque<ElementWithConvertedSize<ros_queue::queue_int_element>>& converted_queue)
    {
        for(deque<ros_queue::queue_int_element>::iterator it = arriving_queue.begin(); it != arriving_queue.end(); ++it)
        {
            int converted_size = sizeof(it->value);

            ElementWithConvertedSize<ros_queue::queue_int_element> convertedElement(std::move(*it), converted_size);
            converted_queue.push_back(convertedElement);
        }
    }
}


TEST_F(RosVirtualQueueFixture, constructorOverrideTest)
{
    // Set mocked_metric
    metric_computation::mocked_metric = 1.5f;

    // InConVirtualQueue queue with user-defined metric computation
    ROSVirtualQueue<InConVirtualQueue> vq0(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                            (ROSVirtualQueue<InConVirtualQueue>::InterfacesArgs){
                                            .metric_computation_fptr= metric_computation::getMockedMetric});
    vq0.update();
    EXPECT_FLOAT_EQ(vq0.getSize(), 1.5f);

    // InConVirtualQueue queue with a service to compute the metric
    ROSVirtualQueue<InConVirtualQueue> vq1(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                            (ROSVirtualQueue<InConVirtualQueue>::InterfacesArgs){
                                            .metric_computation_service_name = metric_computation_service_name_f});
    vq1.update();
    EXPECT_FLOAT_EQ(vq1.getSize(), 1.8f);

    // EqConVirtualQueue queue with user-defined metric computation
    ROSVirtualQueue<EqConVirtualQueue> vq2(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                            (ROSVirtualQueue<EqConVirtualQueue>::InterfacesArgs){
                                            .metric_computation_fptr= metric_computation::getMockedMetric});
    vq2.update();
    EXPECT_FLOAT_EQ(vq2.getSize(), 1.5f);

    // EqConVirtualQueue queue with a service to compute the metric
    ROSVirtualQueue<EqConVirtualQueue> vq3(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                        (ROSVirtualQueue<EqConVirtualQueue>::InterfacesArgs){
                                        .metric_computation_service_name = metric_computation_service_name_f});
    vq3.update();
    EXPECT_FLOAT_EQ(vq3.getSize(), 1.8f);
}

TEST_F(RosVirtualQueueFixture, badInitTest)
{
    //Test all constructors with a bad initialization of their function pointers.

    typedef ROSVirtualQueue<InConVirtualQueue> ROSInConVirtualQUeue; 

    EXPECT_THROW(ROSInConVirtualQUeue vq0(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                            (ROSInConVirtualQUeue::InterfacesArgs){
                                            .metric_computation_fptr= nullptr})
    , invalid_argument);

    EXPECT_THROW(ROSInConVirtualQUeue vq1(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                            (ROSInConVirtualQUeue::InterfacesArgs){
                                            .metric_computation_fptr = nullptr,
                                            .metric_computation_service_name = ""})
    , invalid_argument);

    EXPECT_THROW(ROSInConVirtualQUeue vq2(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                            (ROSInConVirtualQUeue::InterfacesArgs){})
    , invalid_argument);

    typedef ROSVirtualQueue<EqConVirtualQueue> ROSEqConVirtualQUeue; 

    EXPECT_THROW(ROSEqConVirtualQUeue vq3(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                            (ROSEqConVirtualQUeue::InterfacesArgs){
                                            .metric_computation_fptr= nullptr})
    , invalid_argument);

    EXPECT_THROW(ROSEqConVirtualQUeue vq4(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                            (ROSEqConVirtualQUeue::InterfacesArgs){
                                            .metric_computation_fptr = nullptr,
                                            .metric_computation_service_name = ""})
    , invalid_argument);

    EXPECT_THROW(ROSEqConVirtualQUeue vq5(max_queue_size_f, queue_info_f, nh_f, target_metric_f,
                                            (ROSEqConVirtualQUeue::InterfacesArgs){})
    , invalid_argument);

}


TEST_F(RosQueueFixture, constructorOverrideTest)
{
    // ROSQueue queue with user-defined transmission, user-defined arrival and user-defined transmission
    RosQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f,
    (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value_plus_one,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_fptr = transmission::transmission_on_dequeue1});
    vq0.update(arrival_queue_f,0);
    EXPECT_EQ(vq0.evaluate(service_struct_test_f), 5);

    // ROSQueue queue with user-defined transmission, ROS service arrival and user-defined transmission
    RosQueueFixture::ROSIntQueue_f vq1(max_queue_size_f, queue_info_f, nh_f, 
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value_plus_one,
        .transmission_prediction_service_name = transmission_prediction_service_name_f,
        .transmission_fptr = transmission::transmission_on_dequeue1});
    vq1.update(arrival_queue_f,0);
    EXPECT_EQ(vq1.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival and user-defined transmission
    RosQueueFixture::ROSIntQueue_f vq2(max_queue_size_f, queue_info_f, nh_f,
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_service_name = arrival_prediction_service_name_f,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_fptr = transmission::transmission_on_dequeue1});
    vq2.update(arrival_queue_f,0);
    EXPECT_EQ(vq2.evaluate(service_struct_test_f), 6);

    // ROSQueue queue with ROS service transmission, ROS servicearrival and user-defined transmission
    RosQueueFixture::ROSIntQueue_f  vq3(max_queue_size_f, queue_info_f, nh_f, 
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_service_name = arrival_prediction_service_name_f,
        .transmission_prediction_service_name = transmission_prediction_service_name_f,
        .transmission_fptr = transmission::transmission_on_dequeue1});
    vq3.update(arrival_queue_f,0);
    EXPECT_EQ(vq3.evaluate(service_struct_test_f), 3);

    // ROSQueue queue with user-defined transmission, user-defined arrival and transmission topic
    RosQueueFixture::ROSIntQueue_f vq4(max_queue_size_f, queue_info_f, nh_f,
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value_plus_one,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_topic_name = transmission_topic_name_f});
    vq4.update(arrival_queue_f,0);
    EXPECT_EQ(vq4.evaluate(service_struct_test_f), 5);

    // ROSQueue queue with user-defined transmission, ROS service arrival and transmission topic
    RosQueueFixture::ROSIntQueue_f vq5(max_queue_size_f, queue_info_f, nh_f,
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value_plus_one,
        .transmission_prediction_service_name = transmission_prediction_service_name_f,
        .transmission_topic_name = transmission_topic_name_f});
    vq5.update(arrival_queue_f,0);
    EXPECT_EQ(vq5.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival and transmission topic
    RosQueueFixture::ROSIntQueue_f vq6(max_queue_size_f, queue_info_f, nh_f,
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_service_name = arrival_prediction_service_name_f,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_topic_name = transmission_topic_name_f});
    vq6.update(arrival_queue_f,0);
    EXPECT_EQ(vq6.evaluate(service_struct_test_f), 6);

    // ROSQueue queue with ROS service transmission, ROS servicearrival and transmission topic
    RosQueueFixture::ROSIntQueue_f vq7(max_queue_size_f, queue_info_f, nh_f, 
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_service_name = arrival_prediction_service_name_f,
        .transmission_prediction_service_name = transmission_prediction_service_name_f,
        .transmission_topic_name = transmission_topic_name_f});
    vq7.update(arrival_queue_f,0);
    EXPECT_EQ(vq7.evaluate(service_struct_test_f), 3);
}

TEST_F(RosQueueFixture, badInitTest)
{
    //Test all constructors with a bad initialization of their function pointers
    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f,
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = nullptr,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_fptr = transmission::transmission_on_dequeue1})
    , invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f,
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value,
        .transmission_prediction_fptr = nullptr,
        .transmission_fptr = transmission::transmission_on_dequeue1})
    , invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f,
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_fptr = nullptr})
    , invalid_argument);

    EXPECT_THROW(RosQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f,
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){})
    , invalid_argument);
    
}

TEST_F(RosQueueFixture, transmissionTest)
{
    // Clear the test transmission queue is empty
    transmission::output_int_dequeue_1.clear();
    EXPECT_EQ(transmission::output_int_dequeue_1.size(), 0);

    // Test if the user-defined transmission works
    RosQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f,
        (RosQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_fptr = transmission::transmission_on_dequeue1});
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

TEST_F(RosConvertedQueueFixture, constructorOverrideTest)
{
    // ROSQueue queue with user-defined transmission, user-defined arrival, user-defined transmission
    //RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs iargs;
    RosConvertedQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f, 
        (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
            .arrival_prediction_fptr = prediction::return_value_plus_one,
            .transmission_prediction_fptr = prediction::return_value,
            .transmission_fptr = transmission::transmission_on_dequeue1,
            .conversion_fptr = conversion::conversion_int_to_byte
        });
    vq0.update(arrival_queue_f,0);
    EXPECT_EQ(vq0.evaluate(service_struct_test_f), 14);

    // ROSQueue queue with user-defined transmission, ROS service arrival, user-defined transmission
    RosConvertedQueueFixture::ROSIntQueue_f vq1(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                                .arrival_prediction_fptr = prediction::return_value_plus_one,
                                                .transmission_prediction_service_name = transmission_prediction_service_name_f,
                                                .transmission_fptr=transmission::transmission_on_dequeue1,
                                                .conversion_fptr=conversion::conversion_int_to_byte
                                            });
    vq1.update(arrival_queue_f,0);
    EXPECT_EQ(vq1.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival, user-defined transmission
    RosConvertedQueueFixture::ROSIntQueue_f vq2(max_queue_size_f, queue_info_f, nh_f ,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                                .arrival_prediction_service_name=arrival_prediction_service_name_f,
                                                .transmission_prediction_fptr=prediction::return_value,
                                                .transmission_fptr=transmission::transmission_on_dequeue1,
                                                .conversion_fptr=conversion::conversion_int_to_byte
                                                });
    vq2.update(arrival_queue_f,0);
    EXPECT_EQ(vq2.evaluate(service_struct_test_f), 15);

    // ROSQueue queue with ROS service transmission, ROS servicearrival, user-defined transmission
    RosConvertedQueueFixture::ROSIntQueue_f vq3(max_queue_size_f, queue_info_f, nh_f ,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                                .arrival_prediction_service_name=arrival_prediction_service_name_f,
                                                .transmission_prediction_service_name=transmission_prediction_service_name_f,
                                                .transmission_fptr=transmission::transmission_on_dequeue1,
                                                .conversion_fptr=conversion::conversion_int_to_byte});
    vq3.update(arrival_queue_f,0);
    EXPECT_EQ(vq3.evaluate(service_struct_test_f), 3);

    // ROSQueue queue with user-defined transmission, user-defined arrival, transmission topic
    RosConvertedQueueFixture::ROSIntQueue_f vq4(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                                .arrival_prediction_fptr=prediction::return_value_plus_one,
                                                .transmission_prediction_fptr=prediction::return_value,
                                                .transmission_topic_name=transmission_topic_name_f,
                                                .conversion_fptr=conversion::conversion_int_to_byte});
    vq4.update(arrival_queue_f,0);
    EXPECT_EQ(vq4.evaluate(service_struct_test_f), 14);

    // ROSQueue queue with user-defined transmission, ROS service arrival, transmission topic
    RosConvertedQueueFixture::ROSIntQueue_f vq5(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                                .arrival_prediction_fptr=prediction::return_value_plus_one,
                                                .transmission_prediction_service_name=transmission_prediction_service_name_f,
                                                .transmission_topic_name=transmission_topic_name_f,
                                                .conversion_fptr=conversion::conversion_int_to_byte});
    vq5.update(arrival_queue_f,0);
    EXPECT_EQ(vq5.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival, transmission topic
    RosConvertedQueueFixture::ROSIntQueue_f vq6(max_queue_size_f, queue_info_f, nh_f ,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                                .arrival_prediction_service_name=arrival_prediction_service_name_f,
                                                .transmission_prediction_fptr=prediction::return_value,
                                                .transmission_topic_name=transmission_topic_name_f,
                                                .conversion_fptr=conversion::conversion_int_to_byte});
    vq6.update(arrival_queue_f,0);
    EXPECT_EQ(vq6.evaluate(service_struct_test_f), 15);

    // ROSQueue queue with ROS service transmission, ROS servicearrival, transmission topic
    RosConvertedQueueFixture::ROSIntQueue_f vq7(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                            .arrival_prediction_service_name=arrival_prediction_service_name_f,
                                            .transmission_prediction_service_name=transmission_prediction_service_name_f,
                                            .transmission_topic_name=transmission_topic_name_f,
                                            .conversion_fptr=conversion::conversion_int_to_byte});
    vq7.update(arrival_queue_f,0);
    EXPECT_EQ(vq7.evaluate(service_struct_test_f), 3);

    // ===================================
    // ROSQueue queue with user-defined transmission, user-defined arrival, user-defined transmission and
    RosConvertedQueueFixture::ROSIntQueue_f vq8(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                            .arrival_prediction_fptr=prediction::return_value_plus_one,
                                            .transmission_prediction_fptr=prediction::return_value,
                                            .transmission_fptr=transmission::transmission_on_dequeue1,
                                            .conversion_service_name=conversion_service_name_f});
    vq8.update(arrival_queue_f,0);
    EXPECT_EQ(vq8.evaluate(service_struct_test_f), 14);

    // ROSQueue queue with user-defined transmission, ROS service arrival, user-defined transmission and
    RosConvertedQueueFixture::ROSIntQueue_f vq9(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                            .arrival_prediction_fptr=prediction::return_value_plus_one,
                                            .transmission_prediction_service_name=transmission_prediction_service_name_f,
                                            .transmission_fptr=transmission::transmission_on_dequeue1,
                                            .conversion_service_name=conversion_service_name_f});
    vq9.update(arrival_queue_f,0);
    EXPECT_EQ(vq9.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival, user-defined transmission and
    RosConvertedQueueFixture::ROSIntQueue_f vq10(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                            .arrival_prediction_service_name=arrival_prediction_service_name_f,
                                            .transmission_prediction_fptr=prediction::return_value,
                                            .transmission_fptr=transmission::transmission_on_dequeue1,
                                            .conversion_service_name=conversion_service_name_f});
    vq10.update(arrival_queue_f,0);
    EXPECT_EQ(vq10.evaluate(service_struct_test_f), 15);

    // ROSQueue queue with ROS service transmission, ROS servicearrival, user-defined transmission and
    RosConvertedQueueFixture::ROSIntQueue_f vq11(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                            .arrival_prediction_service_name=arrival_prediction_service_name_f,
                                            .transmission_prediction_service_name=transmission_prediction_service_name_f,
                                            .transmission_fptr=transmission::transmission_on_dequeue1,
                                            .conversion_service_name=conversion_service_name_f});
    vq11.update(arrival_queue_f,0);
    EXPECT_EQ(vq11.evaluate(service_struct_test_f), 3);

    // ROSQueue queue with user-defined transmission, user-defined arrival, transmission topic and
    RosConvertedQueueFixture::ROSIntQueue_f vq12(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                            .arrival_prediction_fptr=prediction::return_value_plus_one,
                                            .transmission_prediction_fptr=prediction::return_value,
                                            .transmission_topic_name=transmission_topic_name_f,
                                            .conversion_service_name=conversion_service_name_f});
    vq12.update(arrival_queue_f,0);
    EXPECT_EQ(vq12.evaluate(service_struct_test_f), 14);

    // ROSQueue queue with user-defined transmission, ROS service arrival, transmission topic and
    RosConvertedQueueFixture::ROSIntQueue_f vq13(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                            .arrival_prediction_fptr=prediction::return_value_plus_one,
                                            .transmission_prediction_service_name=transmission_prediction_service_name_f,
                                            .transmission_topic_name=transmission_topic_name_f,
                                            .conversion_service_name=conversion_service_name_f});
    vq13.update(arrival_queue_f,0);
    EXPECT_EQ(vq13.evaluate(service_struct_test_f), 2);

    // ROSQueue queue with ROS service transmission, user-defined arrival, transmission topic and
    RosConvertedQueueFixture::ROSIntQueue_f vq14(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                            .arrival_prediction_service_name=arrival_prediction_service_name_f,
                                            .transmission_prediction_fptr=prediction::return_value,
                                            .transmission_topic_name=transmission_topic_name_f,
                                            .conversion_service_name=conversion_service_name_f});
    vq14.update(arrival_queue_f,0);
    EXPECT_EQ(vq14.evaluate(service_struct_test_f), 15);

    // ROSQueue queue with ROS service transmission, ROS servicearrival, transmission topic and
    RosConvertedQueueFixture::ROSIntQueue_f vq15(max_queue_size_f, queue_info_f, nh_f,
                                            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                                            .arrival_prediction_service_name=arrival_prediction_service_name_f,
                                            .transmission_prediction_service_name=transmission_prediction_service_name_f,
                                            .transmission_topic_name=transmission_topic_name_f,
                                            .conversion_service_name=conversion_service_name_f});
    vq15.update(arrival_queue_f,0);
    EXPECT_EQ(vq15.evaluate(service_struct_test_f), 3);
}

TEST_F(RosConvertedQueueFixture, badInitTest)
{
    //Test all constructors with a bad initialization of their function pointers
        EXPECT_THROW(
        RosConvertedQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f, 
            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){});
    , invalid_argument);

    EXPECT_THROW(
        RosConvertedQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f, 
            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){

                .transmission_prediction_fptr = prediction::return_value,
                .transmission_fptr = transmission::transmission_on_dequeue1,
                .conversion_fptr = conversion::conversion_int_to_byte
            });
    , invalid_argument);

    EXPECT_THROW(
        RosConvertedQueueFixture::ROSIntQueue_f vq1(max_queue_size_f, queue_info_f, nh_f, 
            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                .arrival_prediction_fptr = prediction::return_value_plus_one,

                .transmission_fptr = transmission::transmission_on_dequeue1,
                .conversion_fptr = conversion::conversion_int_to_byte
            });
    , invalid_argument);

    EXPECT_THROW(
        RosConvertedQueueFixture::ROSIntQueue_f vq2(max_queue_size_f, queue_info_f, nh_f, 
            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                .arrival_prediction_fptr = prediction::return_value_plus_one,
                .transmission_prediction_fptr = prediction::return_value,

                .conversion_fptr = conversion::conversion_int_to_byte
            });
    , invalid_argument);

    EXPECT_THROW(
        RosConvertedQueueFixture::ROSIntQueue_f vq3(max_queue_size_f, queue_info_f, nh_f, 
            (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
                .arrival_prediction_fptr = prediction::return_value_plus_one,
                .transmission_prediction_fptr = prediction::return_value,
                .transmission_fptr = transmission::transmission_on_dequeue1

            });
    , invalid_argument);
}

TEST_F(RosConvertedQueueFixture, initOverrideTest)
{
    // Clear the test transmission queue is empty
    transmission::output_int_dequeue_1.clear();
    EXPECT_EQ(transmission::output_int_dequeue_1.size(), 0);

    // Test if function pointers overides the behavior of the service if both are defined
    RosConvertedQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f, 
        (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
            .arrival_prediction_fptr = prediction::return_value_plus_one,
            .arrival_prediction_service_name = arrival_prediction_service_name_f,

            .transmission_prediction_fptr = prediction::return_value,
            .transmission_prediction_service_name = transmission_prediction_service_name_f,

            .transmission_fptr = transmission::transmission_on_dequeue1,
            .transmission_topic_name = transmission_prediction_service_name_f,

            .conversion_fptr = conversion::conversion_int_to_byte,
            .conversion_service_name = conversion_service_name_f
        });

    vq0.update(arrival_queue_f,0);
    // Look if the function
    EXPECT_EQ(vq0.evaluate(service_struct_test_f), 14);

    vq0.update(arrival_queue_f, 2);
    
    deque<ros_queue::queue_int_element> compared_queue;
    ros_queue::queue_int_element int_element;
    int_element.value =1;
    compared_queue.push_back(int_element);
    int_element.value = 2;
    compared_queue.push_back(int_element);

    EXPECT_EQ(transmission::output_int_dequeue_1, compared_queue);

}

TEST_F(RosConvertedQueueFixture, transmissionTest)
{
    // Clear the test transmission queue is empty
    transmission::output_int_dequeue_1.clear();
    EXPECT_EQ(transmission::output_int_dequeue_1.size(), 0);

    // Test if the user-defined transmission works
    RosConvertedQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f, 
    (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value_plus_one,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_fptr = transmission::transmission_on_dequeue1,
        .conversion_fptr = conversion::conversion_int_to_byte,
    });

    vq0.update(arrival_queue_f,0);
    EXPECT_EQ(transmission::output_int_dequeue_1.size(), 0);
    vq0.update(arrival_queue_f, 2);
    EXPECT_EQ(vq0.getSize(), 24);

    
    deque<ros_queue::queue_int_element> compared_queue;
    ros_queue::queue_int_element int_element;
    int_element.value =1;
    compared_queue.push_back(int_element);
    int_element.value = 2;
    compared_queue.push_back(int_element);

    EXPECT_EQ(transmission::output_int_dequeue_1, compared_queue);
    
    // Test if transmission through ROS Topics works. EDIT: Verified in another way.

    /* RosQueueFixture::ROSIntQueue_f vq1(max_queue_size_f, queue_info_f, prediction::return_value_plus_one, prediction::return_value, transmission_topic_name_f);
    vq1.update(arrival_queue_f,0);
    vq1.update(arrival_queue_f, 2);*/

}

TEST_F(RosConvertedQueueFixture, conversionTest)
{
    // Test the user-defined conversion
    RosConvertedQueueFixture::ROSIntQueue_f vq0(max_queue_size_f, queue_info_f, nh_f, 
    (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value_plus_one,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_fptr = transmission::transmission_on_dequeue1,
        .conversion_fptr = conversion::conversion_int_to_byte
    });
    vq0.update(arrival_queue_f,0);
    // Since the conversion::convserion_int_to_byte computes the size in bytes, we can compared it with sizeof()
    EXPECT_EQ(vq0.getSize(), arrival_queue_f.size() * sizeof(ros_queue::queue_int_element));

    //Test the service conversion
    RosConvertedQueueFixture::ROSIntQueue_f vq1(max_queue_size_f, queue_info_f, nh_f, 
    (struct RosConvertedQueueFixture::ROSIntQueue_f::InterfacesArgs){
        .arrival_prediction_fptr = prediction::return_value_plus_one,
        .transmission_prediction_fptr = prediction::return_value,
        .transmission_fptr = transmission::transmission_on_dequeue1,
        .conversion_service_name = conversion_service_name_f
    });
    vq1.update(arrival_queue_f,0);
    EXPECT_EQ(vq1.getSize(), arrival_queue_f.size() * sizeof(ros_queue::queue_int_element));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ros_queue_test");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}