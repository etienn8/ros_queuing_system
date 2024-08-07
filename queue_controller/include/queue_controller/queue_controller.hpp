#pragma once

#include <chrono>
#include <condition_variable>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "ros/ros.h"
#include <ros/callback_queue.h>

#include "controller_queue_struct.hpp"
#include "queue_controller_utils.hpp"

#include "ros_queue_msgs/ControllerActionCosts.h"
#include "ros_queue_msgs/FloatRequest.h"
#include "ros_queue_msgs/GetQueueControllerTiming.h"
#include "ros_queue_msgs/MetricCosts.h"
#include "ros_queue_msgs/QueueControllerTiming.h"
#include "ros_queue_msgs/QueueInfoFetch.h"
#include "ros_queue_msgs/QueueServerStateFetch.h"
#include "ros_queue_msgs/VirtualQueueChangesList.h"
#include "ros_queue_msgs/TransmissionVectorControllerCostsList.h"

#include <actionlib/client/simple_action_client.h>

#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

#include "rosparam_utils/xmlrpc_utils.hpp"
#include "rosparam_utils/parameter_package_fetch_struct.hpp"
#include "rosparam_utils/data_validation.hpp"

#include "ros_boosted_utilities/persistent_service_client.hpp"

#include "queue_server/queue_server_utils.hpp"

using std::string;
using std::map;

template <typename TActionSetType>
struct ActionTrait {
  using ActionType = typename TActionSetType::_action_set_type::value_type;
};

template <typename TControllerCostListType>
struct ActionCostTrait {
  using ActionCostType = typename TControllerCostListType::_action_costs_list_type::value_type;
};

template<typename TMetricControlPredictionSrv, typename TPotentialActionSetMsg, typename TPotentialActionSetSrv, typename TActionLibOutputType, typename TActionLibOutputGoalType, typename TControllerCostListType>
class QueueController
{
    public:
        typedef typename ActionTrait<TPotentialActionSetMsg>::ActionType ActionType;
        typedef typename ActionCostTrait<TControllerCostListType>::ActionCostType ActionCostType;

        enum ControllerType
        {
            DriftPlusPenalty,
            RenewalDriftPlusPenalty
        };

        QueueController(ros::NodeHandle& nh): nhp_(nh), nh_(ros::NodeHandle()), async_spinner_(1, &async_spinner_queue_), async_nhp_("~"), async_nh_(nh_)
        {
            bool can_create_controller = true;

            string controller_type_name;
            if(nhp_.getParam("controller_type", controller_type_name))
            {
                if(controller_type_name == "min_drift_plus_penalty")
                {
                    controller_type_ = ControllerType::DriftPlusPenalty;
                }
                else if (controller_type_name == "renewal_min_drift_plus_penalty")
                {
                    controller_type_ = ControllerType::RenewalDriftPlusPenalty;
                }
                else
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Unknown controller type:" << controller_type_name <<". Supported types: min_drift_plus_penalty and renewal_min_drift_plus_penalty"); 
                }
            }
            else
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the controller type");
            }

            if (nhp_.getParam("is_periodic",is_periodic_))
            {
                if(is_periodic_)
                {
                    ROS_INFO_STREAM("The controller will run periodically.");
                }
                else
                {
                    ROS_INFO_STREAM("The controller will be triggered by a service call.");
                }
            }
            else
            {
                ROS_WARN_STREAM("Missing the is_periodic parameter. The controller will run periodically.");
            }

            if(controller_type_ == ControllerType::DriftPlusPenalty)
            {
                if(is_periodic_ && !nhp_.getParam("time_step", controller_time_step_))
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Missing the controller time step");
                }
            }

            // Configure the async spinner for the asynchronous fetch of the renewal time and synchronization callbacks
            async_nhp_.setCallbackQueue(&async_spinner_queue_);
            async_nh_.setCallbackQueue(&async_spinner_queue_);

            if(controller_type_ == ControllerType::RenewalDriftPlusPenalty)
            {
                if(!nhp_.getParam("max_renewal_time", max_renewal_time_))
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Missing the max_renewal_time");
                }

                if(!nhp_.getParam("min_renewal_time", min_renewal_time_))
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Missing the min_renewal_time");
                }

                if(!nhp_.getParam("is_penalty_renewal_dependent", is_penalty_renewal_dependent_))
                {
                    ROS_WARN_STREAM("Missing the flag is_penalty_renewal_dependent that indicates if the penalty is dependent on the renewal time. Will be set to false.");
                }

                renewal_time_pub_ = nhp_.advertise<ros_queue_msgs::QueueControllerTiming>("renewal_time", 10);

                string expected_renewal_time_service_name;
                if(!nhp_.getParam("expected_renewal_time_service_name", expected_renewal_time_service_name) || expected_renewal_time_service_name.empty())
                {
                    can_create_controller = false;
                    ROS_ERROR_STREAM("Missing the expected_renewal_time_service_name.");
                }
                else
                {
                    renewal_service_client_ = PersistentServiceClient<TMetricControlPredictionSrv>(nh_, expected_renewal_time_service_name);

                    ROS_INFO_STREAM("Waiting for the expected renewal time service named :" << expected_renewal_time_service_name);
                    renewal_service_client_.waitForExistence();
                }

                // Start an asynchronous spinner to handle the last renewal time service in parallel to the controller.
                ROS_INFO_STREAM("Creating the last renewal time service server.");
                last_renewal_time_service_server_ = async_nhp_.advertiseService("get_last_renewal_time", &QueueController::lastRenewalTimeServiceCallback, this);
            }

            if(!nhp_.getParam("inverse_control_and_steps", inversed_control_and_update_steps_))
            {
                ROS_WARN_STREAM("Missing the flag inverse_control_and_steps that indicates if the control and update steps are inverted. Will be set to false.");
            }

            if(!nhp_.getParam("responsible_for_virtual_queue_changes", responsible_for_virtual_queue_changes_))
            {
                ROS_WARN_STREAM("Missing the flag responsible_for_virtual_queue_changes that indicates if the controller should trigger the changes of the virtual queues of the queue server. Will be set to true.");
            }

            string solution_space_service_name;
            if(nhp_.getParam("solution_space_service_name", solution_space_service_name))
            {
                solution_space_client_ = PersistentServiceClient<TPotentialActionSetSrv>(nh_, solution_space_service_name); 
                
                ROS_INFO_STREAM("Waiting for the solution space client named :" << solution_space_service_name);
                solution_space_client_.waitForExistence();
            }
            else
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the solution_space_service_name.");
            }

            string penalty_service_name;
            if(nhp_.getParam("penalty_service_name", penalty_service_name))
            {
                penalty_service_client_ = PersistentServiceClient<TMetricControlPredictionSrv>(nh_, penalty_service_name); 
                
                ROS_INFO_STREAM("Waiting for the penalty space client named :" << penalty_service_name);
                penalty_service_client_.waitForExistence();
            }
            else
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the penalty_service_name.");
            }

            if(nhp_.getParam("v_parameter", v_parameter_))
            {
                if(v_parameter_< 0.0f)
                {
                    ROS_ERROR_STREAM("v_parameter has negative value: "<< v_parameter_ <<". Queue controllers expects non-negative V parameters.");
                }
            }
            else
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the v_parameter.");
            }

            if(nhp_.getParam("measure_cost", measure_cost_))
            {
                if(measure_cost_)
                {
                    cost_publisher_ = nhp_.advertise<TControllerCostListType>("controller_costs", 10);
                }
            }
            else
            {
                ROS_WARN_STREAM("Missing the measure_cost parameter. Will be set to false.");
            }

            if(!nhp_.getParam("queue_server_name", queue_server_name_))
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the queue_server_name.");
            }

            // Parse queue parameters
            xmlrpc_utils::ParameterPackageFetchStruct::OptionsStruct options;

            options.string_parameter_name_list = std::vector<string>{"expected_arrival_service_name", "expected_departure_service_name"};
            options.float_parameter_name_list = std::vector<string>{"weight"};
            options.bool_parameter_name_list = std::vector<string>{"arrival_action_dependent", "departure_action_dependent", "arrival_renewal_dependent", "departure_renewal_dependent"};

            const xmlrpc_utils::ParameterPackageFetchStruct fetch_struct(options);
            
            vector<xmlrpc_utils::ParameterPackageFetchStruct> parsed_queue_configs= 
                            xmlrpc_utils::fetchMatchingParametersFromList(nhp_, ros::this_node::getName(),
                                                                        "queue_list", fetch_struct);

            // Does the queue server exist and transfer queue_configs in internal queue structs
            if(!populateQueueStructures(parsed_queue_configs))
            {
                can_create_controller = false;
            }

            string action_server_name;
            if(!nhp_.getParam("action_server_name", action_server_name))
            {
                can_create_controller = false;
                ROS_ERROR_STREAM("Missing the action_server_name.");
            }

            string dependent_on_controller_topic;
            string start_control_loop_sync_topic;
            if(nhp_.getParam("part_of_multicontroller_synchronization", part_of_multicontroller_synchronization_))
            {
                if (part_of_multicontroller_synchronization_)
                {
                    if(nhp_.getParam("dependent_on_controller_topic",dependent_on_controller_topic))
                    {
                        if (dependent_on_controller_topic.empty())
                        {
                            ROS_INFO_STREAM("Queue controller is not dependant on any topic to send its command. Since its dependent_on_controller_topic parameter is empty.");
                        }
                        else
                        {
                            ROS_INFO_STREAM("Queue controller is dependant on topic "<< dependent_on_controller_topic << " to send its command.");
                            is_dependent_on_a_another_controller_ = true;
                        }
                    }
                    else
                    {
                         ROS_INFO_STREAM("Queue controller is not dependant on any topic to send its command. Since the dependent_on_controller_topic parameter is not set.");
                    }
                    if(nhp_.getParam("start_control_loop_sync_topic", start_control_loop_sync_topic))
                    {
                        if (start_control_loop_sync_topic.empty())
                        {
                            ROS_INFO_STREAM("Queue controller doesn't have a user-defined topic to start its control loop.");
                        }
                        else
                        {
                            ROS_INFO_STREAM("Queue controller uses"<< start_control_loop_sync_topic << " to start its control loop.");
                        }
                    }
                    else
                    {
                         ROS_INFO_STREAM("Queue controller doesn't have a user-defined topic to start its control loop. Since the start_control_loop_sync_topic parameter is not set.");
                    }
                }
            }
            else
            {
                ROS_WARN("The flag part_of_multicontroller_synchronization is not set. It will be set to false.");
            }

            // Intialize controller
            if (can_create_controller)
            {   
                // Connect to queue_server for queue sizes
                server_state_client_ = PersistentServiceClient<ros_queue_msgs::QueueServerStateFetch>(nh_, queue_server_name_ + "/get_server_state");

                ROS_INFO_STREAM("Waiting for the server service named :" << server_state_client_.getService());
                server_state_client_.waitForExistence();

                if(responsible_for_virtual_queue_changes_)
                {
                    // Connect to queue_server for queue server udpates (depends on steps order)
                    if (!inversed_control_and_update_steps_)
                    {   
                        manual_virtual_queue_changes_ = nh_.advertise<ros_queue_msgs::VirtualQueueChangesList>(queue_server_name_ + "/virtual_queue_manual_changes", 1000);
                    }
                    else
                    {
                        virtual_queues_trigger_ = PersistentServiceClient<std_srvs::Empty>(nh_, queue_server_name_ + "/trigger_service"); 
                        ROS_INFO_STREAM("Waiting for the server service named :" << virtual_queues_trigger_.getService());
                        virtual_queues_trigger_.waitForExistence();
                    }
                }

                // Create ouptut topic
                best_action_client_ =  std::make_unique<actionlib::SimpleActionClient<TActionLibOutputType>>(action_server_name, true);   

                if(!is_periodic_)
                {
                    start_control_loop_service_server_ = nhp_.advertiseService("start_control_loop", &QueueController::startControlLoopServiceCallback, this);
                    start_control_loop_sub_ = nhp_.subscribe("start_control_loop", 1, &QueueController::startControlLoopCallback, this);
                }

                if(part_of_multicontroller_synchronization_)
                {
                    if(is_dependent_on_a_another_controller_)
                    {
                        dependent_controller_finished_sub_ = async_nh_.subscribe(dependent_on_controller_topic, 1, &QueueController::dependentControllerFinishedCallback, this);
                        
                        while(dependent_controller_finished_sub_.getNumPublishers() == 0)
                        {
                            ROS_WARN_STREAM_THROTTLE(2, "Waiting for the dependent controller to be online. On topic "<< dependent_controller_finished_sub_.getTopic());
                            ros::Duration(0.1).sleep();
                        }
                    }
                    optimization_done_pub_ = nhp_.advertise<std_msgs::Empty>("optimization_done", 10);
                    control_loop_started_pub_ = nhp_.advertise<std_msgs::Empty>("control_loop_started", 1, true);

                    if(!is_periodic_ && !start_control_loop_sync_topic.empty())
                    {
                        start_control_loop_sync_sub_ = nh_.subscribe(start_control_loop_sync_topic, 1, &QueueController::startControlLoopCallback, this);
                    }
                }

                if(controller_type_ == ControllerType::RenewalDriftPlusPenalty || 
                  (part_of_multicontroller_synchronization_ && is_dependent_on_a_another_controller_))
                {
                    async_spinner_.start();
                }

                ROS_INFO("Queue Controller started");
                is_initialized_ = true;
            }
        }

        struct ObjectiveParameter
        {
            float expected_arrivals;
            float expected_departures;

            float current_size;
        };

        struct ActionParameters
        {
            ActionType action;
            
            float penalty;
            
            std::map<string, ObjectiveParameter> queue_parameters;

            float expected_renewal_time;

            float cost = 0.0f;
        };

        /**
         * @brief Main loop of the queue controller. It will call the control sequence based on the controller type.
        */
        void spin()
        {
            if (!is_initialized_)
            {
                ROS_ERROR_STREAM_THROTTLE(2, "Queue controller wasn't initialized. Stopping queue controller.");
                return;
            }

            if(is_periodic_)
            {
                if (controller_type_ == ControllerType::DriftPlusPenalty)
                {
                   ros::Rate loop_rate(1.0f/controller_time_step_);
                    while(ros::ok())
                    {
                        controllerCallback();
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }
                else if (controller_type_ == ControllerType::RenewalDriftPlusPenalty)
                {
                    while(ros::ok())
                    {
                        if(!best_action_client_waited_)
                        {
                            if(!best_action_client_->waitForServer(ros::Duration(5.0)))
                            {
                                ROS_WARN_STREAM("The action server named wasn't found. Queue controller will be running with a perdiod of max_renewal_time ("<< max_renewal_time_ << "s) between each control step and will continue to try to connect to server.");
                            }

                            best_action_client_waited_ = true;
                        }

                        // Start to measure how much time the controlle takes time to exectue
                        ros::Time controller_start_time_point = ros::Time::now();

                        ros_queue_msgs::QueueControllerTiming renewal_time_msg;
                        // Access protection to the last_renewal_time_ variable
                        {
                            std::lock_guard<std::mutex> lock(last_renewal_time_mutex_);
                            if(is_first_renewal_loop_)
                            {
                                is_first_renewal_loop_ = false;
                                last_renewal_time_point = ros::Time::now();
                                last_controller_execution_time_ = 0;
                            }

                            // Compute the real elapsed time since the last renewal.
                            last_renewal_time_ = (ros::Time::now() - last_renewal_time_point).toSec();
                            
                            renewal_time_msg.renewal_time = last_renewal_time_;
                            renewal_time_msg.execution_time = last_controller_execution_time_;
                            ROS_DEBUG_STREAM("Time since last renewal: " << last_renewal_time_);
                        }
                        renewal_time_pub_.publish(renewal_time_msg);
                        
                        // Compute the controller
                        controllerCallback();
                        
                        {
                            // Measure how much time the controlle took to execute.
                            std::lock_guard<std::mutex> lock(last_renewal_time_mutex_);
                            last_controller_execution_time_ = (ros::Time::now() - controller_start_time_point).toSec();
                        }

                        last_renewal_time_point = ros::Time::now();

                        if(best_action_client_->isServerConnected())
                        {
                            // Wait for the best action to be reached or wait for the max_renewal_time
                            bool finished_before_max_time =  best_action_client_->waitForResult(ros::Duration(max_renewal_time_));
                            
                            // We abandon the last goal if its still on going.
                            if (!finished_before_max_time)
                            {
                                best_action_client_->cancelGoal();
                            }

                            double elapsed_time = (ros::Time::now() - last_renewal_time_point).toSec();

                            // Wait for t_min is reached if the goal was reached before.
                            if (elapsed_time < min_renewal_time_)
                            {
                                ros::Duration(min_renewal_time_ - elapsed_time).sleep();
                            }
                        }
                        else
                        {
                            ros::Duration(max_renewal_time_).sleep();      
                        }
                    }
                }
            }
            else
            {
                // If non periodic, a service is used to start a control loop.
                ros::spin();
            }
        }

        bool startControlLoopServiceCallback(std_srvs::Empty::Request& req, 
                                             std_srvs::Empty::Response& res)
        {
            ROS_DEBUG_STREAM("Starting control loop from service call for the controller: " << ros::this_node::getName());

            controllerCallback();
            return true;
        }

    private:
        /**
         * @brief Indicates if the controller received a valid configuration and was initialized.
        */
        bool is_initialized_ = false;

        /**
         * @brief Node handle with the namespace prefix of the namespace of the node. Mainly used for the declaration of services
         * for user-defined services.
        */
        ros::NodeHandle nh_;

        /**
         * @brief Private node handle with namespace prefixed with the namespace of the node and it's name. Mainly used for declaration 
         * of topics and services provided for the node.
        */
        ros::NodeHandle nhp_;
        
        /**
         * @brief Pivate Nodehandle that uses the async_spinner_queue_ as its callback queue. It's use as an interface
         * to create services from that should be manage in parallel with the main thread.
        */
        ros::NodeHandle async_nhp_;

        /**
         * @brief Pivate Nodehandle that uses the async_spinner_queue_ as its callback queue. It's use as an interface
         * to create services from that should be manage in parallel with the main thread.
        */
        ros::NodeHandle async_nh_;

        // ===== Synchronization parameters ======

        /**
         * @brief Flag that indicates if the controller should run periodically or should be triggered by a service call.
        */
        bool is_periodic_ = true;

        /** */
        /**
         * @brief ROS Service Server that launches a control loop if the flag is_periodic_ is set to false; 
        */
        ros::ServiceServer start_control_loop_service_server_;

        /**
         * @brief Flag that indicates if the queue controller is part of a multi-controller system
         * that are synchronized between them.
        */
        bool part_of_multicontroller_synchronization_ = false;

        /**
         * @brief Flag that indicates if the controller should wait for another controller before continuing.
        */
        bool is_dependent_on_a_another_controller_ = false;

        /**
         * @brief ROS publisher that indicates the begining of the control loop. Mainly used by the director controller
         * to send a control signal to all dependent controllers to start their control loops.
        */
        ros::Publisher control_loop_started_pub_;

        /**
         * @brief ROS Subscriber that when called, the controller will start a control loop. Similar to start_control_loop_service_server_.
         * It will be used only if is_periodic_ is set to false.
        */
        ros::Subscriber start_control_loop_sub_;

        /**
         * @brief ROS Subscriber that when called, the controller will start a control loop. Its a duplication of the start_control_loop_sub_
         * bu explicitely used for the synchronization and can be defined by the user.
         * It will be used only if is_periodic_ is set to false.
        */
        ros::Subscriber start_control_loop_sync_sub_;

        /**
         * @brief Callback for the start_control_loop_sub_ that will start a control loop based on the reception of a empty topic.
         * @param msg Empty message that indicates the start of the control loop.
        */
        void startControlLoopCallback(const std_msgs::Empty::ConstPtr& msg)
        {
            if (is_dependent_on_a_another_controller_)
            {
                std::lock_guard<std::mutex> lock(is_waiting_for_other_controller_mutex_access);
                is_waiting_for_other_controller_ = true;
            }

            ROS_DEBUG_STREAM("Starting control loop from topic for the controller: " << ros::this_node::getName());
            controllerCallback();
        }

        /**
         * @brief Flag that indicates if the responder controller is waiting for another controller to finish before sending its action.
        */
        bool is_waiting_for_other_controller_ = false;

        /**
         * @brief ROS Subscriber that listens for dependent queue to finish and signal this controller when it happens.
        */
        ros::Subscriber dependent_controller_finished_sub_;

        /**
         * @brief ROS Publisher that sends a empty message to signal that this controller has finished and it's 
         * dependent controller has signal it to continue. 
        */
        ros::Publisher optimization_done_pub_;

        /**
         * @brief Mutex that protects access to the is_waiting_for_other_controller_ variable.
        */
        std::mutex is_waiting_for_other_controller_mutex_access;

        /**
         * @brief Mutex used to signal and notify when a dependent controller finished.
        */
        std::mutex controller_synchronization_mutex_;

        /**
         * @brief Condition variable that verifies if a dependent controller finished its optimization.
        */
        std::condition_variable controller_synchronization_cv_;

        /**
         * @brief Starting method of the synchronization mechanimim called at the begening of the controller to 
         * ppublish and emptry start message and set the is_waiting_for_other_controller_ flag to true if 
         * the is_dependent_on_a_another_controller_ flag is true.
        */
        void startSynchronization()
        {
            std_msgs::Empty empty_msg;
            control_loop_started_pub_.publish(empty_msg);

            if(is_dependent_on_a_another_controller_)
            {
                std::lock_guard<std::mutex> lock(is_waiting_for_other_controller_mutex_access);
                is_waiting_for_other_controller_ = true;
            }
        }

        /**
         * @brief Wait for the dependent controller to finish its optimization by waiting 
         * for the is_waiting_for_other_controller_ flag to be set to false by the callback of the dependent_controller_finished_sub_.
        */
        void waitForSynchronization()
        {
            if(is_waiting_for_other_controller_)
            {
                ROS_DEBUG_STREAM("Waiting for dependent controller to finish its optimization. On topic "<< dependent_controller_finished_sub_.getTopic());
                std::unique_lock<std::mutex> lock(controller_synchronization_mutex_);
                while(is_waiting_for_other_controller_ && ros::ok())
                {
                    // This perdiodic wait_for() is used instead of wait() to allow the thread to detect if 
                    // ROS has received a signal to shutdown. Otherwise, the thread would be blocked until a SIGTERM is called. 
                    controller_synchronization_cv_.wait_for(lock, std::chrono::seconds(1), [this]{return (!this->is_waiting_for_other_controller_ || !ros::ok());});
                }
                ROS_DEBUG_STREAM("Finished waiting for dependent controller");
            }
        }

        /**
         * @brief Callback for the dependent_controller_finished_sub_ that will signal the controller that the controller
         * it's depending from finish its optimation. Thus allowing this controller to continue to send its best action.
         * @param msg Empty message that indicates the dependent controller finished its optimization.
        */
        void dependentControllerFinishedCallback(const std_msgs::Empty::ConstPtr& msg)
        {
            {
                std::lock_guard<std::mutex> lock(controller_synchronization_mutex_);
                is_waiting_for_other_controller_ = false;
            }
            controller_synchronization_cv_.notify_one();
        }

        // ===== Renewwal parameters ======

        /**
         * @brief Time at which the last renewal was done. Used by the renewal_min_drift_plus_penalty controller.
        */
        ros::Time last_renewal_time_point;

        /**
         * @brief Duration of the last execution of the controller
        */
        float last_controller_execution_time_;

        /**
         * @brief Last elapsed time between the last renewal and the current time in seconds. Used by the renewal_min_drift_plus_penalty controller
         * and only used with virtual updates that are based on the current state of the system (inversed_control_and_update_steps_ is set to true).
        */
        double last_renewal_time_;

        /**
         * @brief ROS service client used to computed the expected time ot execute each given action.
        */
        PersistentServiceClient<TMetricControlPredictionSrv> renewal_service_client_;

        /**
         * @brief ROS topic publisher that periodically sends the last renewal time.
        */
        ros::Publisher renewal_time_pub_;

        /**
         * @brief Flag used to know if the first control loop of the renewal controller has been done.
        */
        bool is_first_renewal_loop_ = true;

        /**
         * @brief ROS service client used to get the current states of the queues in the queue server.
        */
        PersistentServiceClient<ros_queue_msgs::QueueServerStateFetch> server_state_client_;

        /**
         * @brief ROS topic publisher use to send manual changes to the virtual queues of a queue server.
         * It's mainly used when the inversed_control_and_update_steps_ is set to false since the queues must change
         * based on the metric computed for the best action.
        */
        ros::Publisher manual_virtual_queue_changes_;

        /**
         * @brief ROS Service client that whenever it's called, the virtual queues in the queue server
         * will update based on the current system's state.
        */
        PersistentServiceClient<std_srvs::Empty> virtual_queues_trigger_;

        /**
         * @brief Action lib client that sends the optimal action to a server.
        */
         std::unique_ptr<actionlib::SimpleActionClient<TActionLibOutputType>> best_action_client_;

        /**
         * @brief Flag that indicates if the best action client already waited for the server to be online. 
        */
        bool best_action_client_waited_ = false; 

        /**
         * @brief Publisher that publishes the cost of each action and all its internal metrics.
        */
        ros::Publisher cost_publisher_;

        /**
         * @brief Flag that indicates if we should measure and publish the cost of the controller steps.
        */
        bool measure_cost_ = false;

        /**
         * @brief Map of all the queues used by the queue_controller from a queue_server. The key
         * is the queue's name and the value is all the usefull parameters of the queue to evaluate and 
         * udpate the queues.
        */
        map<string, std::unique_ptr<ControllerQueueStruct<TMetricControlPredictionSrv>>> queue_list_;

        /**
         * @brief Populate the internal queue_list_ based on the parsed_queue_configs. It will also verify
         * if the queues exist in the queue server. If a queue doesn't exist, it won't be added to the queue_list.
         * @param parsed_queue_configs Parsed parameters from the queue_controller configuration from which
         * the queue_list_ will be populated.
         * @return Returns false if the queue server wasn't found and true otherwise.
        */
        bool populateQueueStructures(vector<xmlrpc_utils::ParameterPackageFetchStruct>& parsed_queue_configs)
        {
            // Create structure to parse the queue params from the queue server
            ROS_INFO_STREAM("Waiting for queue serve named " << queue_server_name_ << " to be online. Waiting at most 10 seconds.");
            
            if (ros::service::waitForService(queue_server_name_+"/get_server_state", ros::Duration(10)))
            {
                ROS_INFO_STREAM("Queue server found. Resuming queue controller initialization.");

                for(auto parsed_queue_config_it = parsed_queue_configs.begin(); parsed_queue_config_it != parsed_queue_configs.end(); ++parsed_queue_config_it)
                {
                    const string& queue_name_temp = parsed_queue_config_it->param_package_name_;
                    
                    ros_queue_msgs::QueueInfoFetch queue_info_srv;

                    if(ros::service::call(queue_server_name_+"/"+queue_name_temp+"/getQueueInfo", queue_info_srv))
                    {
                        auto new_controller_struct = std::make_unique<ControllerQueueStruct<TMetricControlPredictionSrv>>();

                        new_controller_struct->queue_name_ = queue_name_temp;
                        new_controller_struct->is_virtual_ = queue_info_srv.response.info.is_virtual;
                        
                        bool is_queue_parameters_ok = true;

                        // Verify if the weight parameter of a queue is set 
                        auto weight_pair_it = parsed_queue_config_it->float_params_.find("weight");
                        if (weight_pair_it != parsed_queue_config_it->float_params_.end() && weight_pair_it->second.second)
                        {
                            new_controller_struct->weight_ = weight_pair_it->second.first;
                        }
                        else
                        {
                            is_queue_parameters_ok = false;
                            ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing his weight parameter");
                        }

                        // Verify if the expected_arrival_service_name is set
                        auto service_name_pair_it = parsed_queue_config_it->string_params_.find("expected_arrival_service_name");
                        if (service_name_pair_it != parsed_queue_config_it->string_params_.end() && service_name_pair_it->second.second)
                        {
                            string& service_name_temp = service_name_pair_it->second.first;
                            
                            // Verify if the arrivals of the queue is affected by the renewal time.
                            auto arrival_renewal_flag_it = parsed_queue_config_it->bool_params_.find("arrival_renewal_dependent");
                            if (controller_type_ == ControllerType::RenewalDriftPlusPenalty && 
                                arrival_renewal_flag_it != parsed_queue_config_it->bool_params_.end() && 
                                arrival_renewal_flag_it->second.second)
                            {
                                new_controller_struct->is_arrival_renewal_dependent = arrival_renewal_flag_it->second.first;
                            }

                            // Verify if it should subscribe to a srv with an action request.
                            auto action_dependent_flag_it = parsed_queue_config_it->bool_params_.find("arrival_action_dependent");
                            if (action_dependent_flag_it != parsed_queue_config_it->bool_params_.end())
                            {
                                if(action_dependent_flag_it->second.first)
                                {
                                    new_controller_struct->is_arrival_action_dependent = true;
                                    new_controller_struct->expected_arrival_service_ = 
                                                            PersistentServiceClient<TMetricControlPredictionSrv>(nh_, service_name_temp);
                                    ROS_INFO_STREAM("Waiting for the server service named :" << new_controller_struct->expected_arrival_service_.getService());
                                    new_controller_struct->expected_arrival_service_.waitForExistence();
                                }
                                else
                                {
                                    // The parameter was not found in the rosparam file
                                    if (!action_dependent_flag_it->second.second)
                                    {
                                        ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing its arrival_action_dependent parameter. Will be set to false.");
                                    }
                                    new_controller_struct->is_arrival_action_dependent = false;
                                    new_controller_struct->arrival_independent_from_action_service_ = 
                                                            PersistentServiceClient<ros_queue_msgs::FloatRequest>(nh_, service_name_temp);
                                    ROS_INFO_STREAM("Waiting for the server service named :" << new_controller_struct->arrival_independent_from_action_service_.getService());
                                    new_controller_struct->arrival_independent_from_action_service_.waitForExistence();
                                }
                            }
                        }
                        else
                        {
                            is_queue_parameters_ok = false;
                            ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing his expected_arrival_service_name parameter");
                        }

                        // Verify if the expected_departure_service_name is set
                        service_name_pair_it = parsed_queue_config_it->string_params_.find("expected_departure_service_name");
                        if (service_name_pair_it != parsed_queue_config_it->string_params_.end() && service_name_pair_it->second.second)
                        {
                            string& service_name_temp = service_name_pair_it->second.first;
                            
                            // Verify if the departures of the queue is affected by the renewal time.
                            auto departure_renewal_flag_it = parsed_queue_config_it->bool_params_.find("departure_renewal_dependent");
                            if (controller_type_ == ControllerType::RenewalDriftPlusPenalty && 
                                departure_renewal_flag_it != parsed_queue_config_it->bool_params_.end() && 
                                departure_renewal_flag_it->second.second)
                            {
                                new_controller_struct->is_departure_renewal_dependent = departure_renewal_flag_it->second.first;
                            }

                            // Verify if it should subscribe to a srv with an action request.
                            auto action_dependent_flag_it = parsed_queue_config_it->bool_params_.find("departure_action_dependent");
                            if (action_dependent_flag_it != parsed_queue_config_it->bool_params_.end())
                            {
                                if(action_dependent_flag_it->second.first)
                                {
                                    new_controller_struct->is_departure_action_dependent= true;
                                    new_controller_struct->expected_departure_service_ = 
                                                            PersistentServiceClient<TMetricControlPredictionSrv>(nh_, service_name_temp);
                                    ROS_INFO_STREAM("Waiting for the server service named :" << new_controller_struct->expected_departure_service_.getService());
                                    new_controller_struct->expected_departure_service_.waitForExistence();
                                }
                                else
                                {
                                    // The parameter was not found in the rosparam file
                                    if (!action_dependent_flag_it->second.second)
                                    {
                                        ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing its departure_action_dependent parameter. Will be set to false.");
                                    }
                                    new_controller_struct->is_departure_action_dependent= false;
                                    new_controller_struct->departure_independent_from_action_service_ = 
                                                            PersistentServiceClient<ros_queue_msgs::FloatRequest>(nh_, service_name_temp);
                                    ROS_INFO_STREAM("Waiting for the server service named :" << new_controller_struct->expected_departure_service_.getService());
                                    new_controller_struct->expected_departure_service_.waitForExistence();
                                    
                                }
                            }

                        }
                        else
                        {
                            is_queue_parameters_ok = false;
                            ROS_WARN_STREAM("The queue "<< queue_name_temp << " from the queue controller configuration is missing his expected_arrival_service_name parameter");
                        }

                        // Add queue to internal structure if all necessary parameters are defined
                        if(is_queue_parameters_ok)
                        {
                            queue_list_[queue_name_temp] = std::move(new_controller_struct);
                        }
                    }
                    else
                    {
                        ROS_WARN_STREAM("Expected queue " << queue_name_temp << " from the queue controller wasn't found in the queue server " << queue_server_name_ << ". The queue won't be considered by the controller.");
                    }
                }
            }
            else
            {
                ROS_ERROR_STREAM("Queue server not found. Stopping queue controller initialization.");
                return false;
            }

            return true;
        }

        /**
         * @brief Service callbacks queue to manage in parallel of the queue controller main loop the 
         * request for the last renewal time.
        */
        ros::CallbackQueue async_spinner_queue_;

        /**
         * @brief Asynchronous spinner that calls the callbacks in the async_spinner_queue_ in parallel of the main
         * thread.
        */
        ros::AsyncSpinner async_spinner_;

        /**
         * @brief Service server that returns the last renewal time. Used with renewal system.
         * Mainly called by nodes that need to update the virtual queues based on the current state of the system.
        */
        ros::ServiceServer last_renewal_time_service_server_;

        /**
         * @brief Service callback that returns the last renewal time. Used with renewal system.
        */
        bool lastRenewalTimeServiceCallback(ros_queue_msgs::GetQueueControllerTiming::Request& req, 
                                            ros_queue_msgs::GetQueueControllerTiming::Response& res)
        {
            std::lock_guard<std::mutex> lock(last_renewal_time_mutex_);
            res.timing.renewal_time = last_renewal_time_;
            res.timing.execution_time = last_controller_execution_time_;
            return true;
        }

        /**
         * @brief Mutex that protects access to the last_renewal_time_ variable.
        */
        std::mutex last_renewal_time_mutex_;

        // ===== Controller steps ======
        /**
         * @brief Callback of called from the spin to exectute all the steps from the controller.
        */
        void controllerCallback()
        {
            // Time measurement variables
            std::chrono::time_point<std::chrono::system_clock> time_0 = std::chrono::high_resolution_clock::now();
            std::chrono::time_point<std::chrono::system_clock> time_cursor = time_0;
            double virtual_queues_current_state_update_time_spent = 0.0f;
            double action_set_time_spent = 0.0f;
            double get_parameters_time_spent = 0.0f;
            double compute_optimization_time_spent = 0.0f;
            double send_best_action_time_spent = 0.0f;
            double update_virtual_queues_time_spent = 0.0f; 
            double total_time = 0.0f;
            
            ROS_DEBUG_STREAM("Queue controller: Starting control loop of "<< ros::this_node::getName());
            
            if (inversed_control_and_update_steps_ && responsible_for_virtual_queue_changes_)
            {
                updateVirtualQueuesBasedOnCurrentState();
            }
            time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, virtual_queues_current_state_update_time_spent);

            if(part_of_multicontroller_synchronization_)
            {
                startSynchronization();
            }

            TPotentialActionSetMsg action_set =  getActionSet();
            time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, action_set_time_spent);

            int nb_actions = action_set.action_set.size();
            if (nb_actions > 0)
            {
                std::vector<ActionParameters> action_parameters_list(nb_actions);
                if(getParametersForControlStep(action_set, action_parameters_list))
                {
                    time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, get_parameters_time_spent);
                    
                    ActionParameters best_action_parameters;
                    if (controller_type_ == ControllerType::DriftPlusPenalty)
                    {
                        best_action_parameters = computeMinDriftPlusPenalty(action_parameters_list);
                    }
                    else if (controller_type_ == ControllerType::RenewalDriftPlusPenalty)
                    {
                        best_action_parameters = computeRenewalMinDriftPlusPenalty(action_parameters_list);
                    }

                    time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, compute_optimization_time_spent);
                
                    if(part_of_multicontroller_synchronization_)
                    {
                        if(is_dependent_on_a_another_controller_)
                        {
                            waitForSynchronization();
                        }
                        optimization_done_pub_.publish(std_msgs::Empty());
                    }

                    sendBestCommand(best_action_parameters.action);
                    time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, send_best_action_time_spent);

                    if (!inversed_control_and_update_steps_ && responsible_for_virtual_queue_changes_)
                    {
                        updateVirtualQueuesBasedOnBestAction(best_action_parameters);
                    }
                    time_cursor = queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_cursor, update_virtual_queues_time_spent);
                }
            }

            queue_controller_utils::updateTimePointAndGetTimeDifferenceMS(time_0, total_time);
            bool can_display_iddle_time = true;
            if (data_validation::float_compare(controller_time_step_, 0.0f))
            {
                can_display_iddle_time = false;
            }
            else if (controller_type_ == ControllerType::RenewalDriftPlusPenalty && data_validation::float_compare(min_renewal_time_, 0.0f))
            {
                can_display_iddle_time = false;
            }

            if(can_display_iddle_time)
            {
                double controller_time_step_ms;
                if (controller_type_ == ControllerType::RenewalDriftPlusPenalty)
                {
                    controller_time_step_ms = min_renewal_time_*1000.0;
                }
                else
                {
                    controller_time_step_ms = controller_time_step_*1000.0;
                }
                ROS_DEBUG_STREAM("Queue controller: Control loop of "<< ros::this_node::getName() << " took " << total_time << " ms to complete. Time spent in each step: \n" 
                                << "virtual_queues_current_state_update_time_spent: " << virtual_queues_current_state_update_time_spent << " ms (" << virtual_queues_current_state_update_time_spent/total_time*100<< "%), \n" 
                                << "action_set_time_spent: " << action_set_time_spent << " ms (" << action_set_time_spent/total_time*100<< "%), \n" 
                                << "get_parameters_time_spent: " << get_parameters_time_spent << " ms (" << get_parameters_time_spent/total_time*100<< "%), \n" 
                                << "compute_optimization_time_spent: " << compute_optimization_time_spent << " ms (" << compute_optimization_time_spent/total_time*100<< "%), \n"
                                << "send_best_action_time_spent: " << send_best_action_time_spent << " ms (" << send_best_action_time_spent/total_time*100<< "%), \n"
                                << "update_virtual_queues_time_spent: " << update_virtual_queues_time_spent << " ms (" << update_virtual_queues_time_spent/total_time*100<< "%), \n"
                                << "iddle time (based on tmin for renewal):" << controller_time_step_ms - total_time << " ms (" << (controller_time_step_ms - total_time)/controller_time_step_ms*100<< "%)");
            }
            else
            {
                ROS_DEBUG_STREAM("Queue controller: Control loop of "<< ros::this_node::getName() << " took " << total_time << " ms to complete. Time spent in each step: \n" 
                                << "virtual_queues_current_state_update_time_spent: " << virtual_queues_current_state_update_time_spent << " ms (" << virtual_queues_current_state_update_time_spent/total_time*100<< "%), \n" 
                                << "action_set_time_spent: " << action_set_time_spent << " ms (" << action_set_time_spent/total_time*100<< "%), \n" 
                                << "get_parameters_time_spent: " << get_parameters_time_spent << " ms (" << get_parameters_time_spent/total_time*100<< "%), \n" 
                                << "compute_optimization_time_spent: " << compute_optimization_time_spent << " ms (" << compute_optimization_time_spent/total_time*100<< "%), \n"
                                << "send_best_action_time_spent: " << send_best_action_time_spent << " ms (" << send_best_action_time_spent/total_time*100<< "%), \n"
                                << "update_virtual_queues_time_spent: " << update_virtual_queues_time_spent << " ms (" << update_virtual_queues_time_spent/total_time*100<< "%), \n");
            }
        }

        /**
         * @brief Calls the service to get the all the potential actions that will be evaluated.
         * @return The set of potential actions.
        */
        TPotentialActionSetMsg getActionSet()
        {
            ROS_DEBUG("Calling action set service");
            
            TPotentialActionSetSrv potential_set_srv;
            if(!solution_space_client_.call(potential_set_srv))
            {
                ROS_WARN_STREAM_THROTTLE(2, "Failed to call the service that retrieves the action set");
            }

            return potential_set_srv.response.action_set;
        }

        /**
         * @brief Enumeration that help to differentiate the type of the parameters to get for the control step.
        */
        enum ParameterType
        {
            Penalty=0,
            RenewalTime,
            Arrival,
            Departure
        };

        /**
         * @brief Structure that holds the information to call a service, to hold the input and the output of 
         * asynchronous service calls.
         * @tparam TSrvType The type of the service that will be called asynchronously.
        */
        template<typename TSrvType>
        struct AsyncMetricFutureStruct
        {
            std::shared_ptr<PersistentServiceClient<TSrvType>> client_;
            TSrvType current_srv_msg_;
            bool current_call_succeeded_;

            string queue_name_;

            ParameterType parameter_type_;
        };

        /**
         * @brief Adds an asynchronous service call to a future array where the definition of the service call is TMetricControlPredictionSrv.
         * @param client The service client that will be used to call the service.
         * @param action_set_msg The action set that will be used as input for the service call.
         * @param parameter_type The type of the parameter to fetch. It could be the penalty, the renewal time, 
         * the arrival or the departure.
         * @param queue_name The name of the queue that will be used as input for the service call. It's only used
         * if the parameter_type is arrival or departure.
        */
        void addServiceCallToActionFutureArray(std::vector<std::future<std::shared_ptr<AsyncMetricFutureStruct<TMetricControlPredictionSrv>>>>& future_array, 
                            PersistentServiceClient<TMetricControlPredictionSrv>& client,
                            const TPotentialActionSetMsg& action_set_msg,
                            ParameterType parameter_type,
                            string queue_name = "")
        {
            auto temp_metric_control_future_struct = std::make_shared<AsyncMetricFutureStruct<TMetricControlPredictionSrv>>();
            temp_metric_control_future_struct->client_ = std::make_shared<PersistentServiceClient<TMetricControlPredictionSrv>>(client);
            temp_metric_control_future_struct->current_srv_msg_.request.action_set = action_set_msg;
            temp_metric_control_future_struct->parameter_type_ = parameter_type;

            if (parameter_type == ParameterType::Arrival || parameter_type == ParameterType::Departure)
            {
                temp_metric_control_future_struct->queue_name_ = queue_name;
            }

            future_array.push_back(std::async(std::launch::async, [](std::shared_ptr<AsyncMetricFutureStruct<TMetricControlPredictionSrv>> async_future_struct){
                if (async_future_struct->client_->call(async_future_struct->current_srv_msg_))
                {
                    async_future_struct->current_call_succeeded_ = true;
                }
                else
                {
                    async_future_struct->current_call_succeeded_ = false;
                }
                return async_future_struct;
            }, temp_metric_control_future_struct));
        }

        /**
         * @brief Adds an asynchronous service call to a future array where the definition of the service call is ros_queueu_msgs::FloatRequest.
         * @param client The service client that will be used to call the service.
         * @param action_set_msg The action set that will be used as input for the service call.
         * @param parameter_type The type of the parameter to fetch. It could be the penalty, the renewal time, 
         * the arrival or the departure.
         * @param queue_name The name of the queue that will be used as input for the service call. It's only used
         * if the parameter_type is arrival or departure.
        */
        void addServiceCallToFloatFutureArray(std::vector<std::future<std::shared_ptr<AsyncMetricFutureStruct<ros_queue_msgs::FloatRequest>>>>& future_array, 
                            PersistentServiceClient<ros_queue_msgs::FloatRequest>& client,
                            const TPotentialActionSetMsg& action_set_msg,
                            ParameterType parameter_type,
                            string queue_name = "")
        {
            auto temp_metric_control_future_struct = std::make_shared<AsyncMetricFutureStruct<ros_queue_msgs::FloatRequest>>();
            temp_metric_control_future_struct->client_ = std::make_shared<PersistentServiceClient<ros_queue_msgs::FloatRequest>>(client);
            temp_metric_control_future_struct->parameter_type_ = parameter_type;

            if (parameter_type == ParameterType::Arrival || parameter_type == ParameterType::Departure)
            {
                temp_metric_control_future_struct->queue_name_ = queue_name;
            }

            future_array.push_back(std::async(std::launch::async, [](std::shared_ptr<AsyncMetricFutureStruct<ros_queue_msgs::FloatRequest>> async_future_struct){
                if (async_future_struct->client_->call(async_future_struct->current_srv_msg_))
                {
                    async_future_struct->current_call_succeeded_ = true;
                }
                else
                {
                    async_future_struct->current_call_succeeded_ = false;
                }
                return async_future_struct;
            }, temp_metric_control_future_struct));
        }
        
        /**
         * @brief Evaluates the penalty, the queues sizes and the queue changes for each action.
         * @param action_set The set of potential actions from which all metrics will be predicted.
         * @param action_parameters_output List of all the penalty and queues parameters for each action. Used as an output.
         * @return Returns true if the parameters are valid.
        */
         bool getParametersForControlStep(TPotentialActionSetMsg& action_set_msg, std::vector<ActionParameters>& action_parameters_output)
        {
            const int& size_of_actions = action_set_msg.action_set.size();

            // Populate actions
            for (int action_index = 0; action_index < size_of_actions; ++action_index)
            {
                action_parameters_output[action_index].action = action_set_msg.action_set[action_index];
            }

            bool are_parameters_valid = true;

            // Get queue sizes
            ros_queue_msgs::QueueServerStateFetch server_state_msg;
            if(!server_state_client_.call(server_state_msg))
            {
                are_parameters_valid = false;
                ROS_WARN_STREAM_THROTTLE(2, "Failed to call the queue server state fetch# service");
            }

            // Create std::future arrays to asynchronously call and wait for the service calls
            std::vector<std::future<std::shared_ptr<AsyncMetricFutureStruct<TMetricControlPredictionSrv>>>> metric_control_future_array;
            std::vector<std::future<std::shared_ptr<AsyncMetricFutureStruct<ros_queue_msgs::FloatRequest>>>> float_request_future_array;

            // Penalty service
            addServiceCallToActionFutureArray(metric_control_future_array, penalty_service_client_, action_set_msg, ParameterType::Penalty);

            if (controller_type_ == ControllerType::RenewalDriftPlusPenalty)
            {
                // Expected time
                addServiceCallToActionFutureArray(metric_control_future_array, renewal_service_client_, action_set_msg, ParameterType::RenewalTime);
            }

            // Queues services
            for (auto queue_it = queue_list_.begin(); queue_it != queue_list_.end(); ++queue_it)
            {
                const string& queue_name = queue_it->first;
                
                // Arrivals
                if(queue_it->second->is_arrival_action_dependent)
                {
                    addServiceCallToActionFutureArray(metric_control_future_array, queue_it->second->expected_arrival_service_, action_set_msg, ParameterType::Arrival, queue_name);
                }
                else
                {
                    addServiceCallToFloatFutureArray(float_request_future_array, queue_it->second->arrival_independent_from_action_service_, action_set_msg, ParameterType::Arrival, queue_name);
                }

                // Departures
                if(queue_it->second->is_departure_action_dependent)
                {
                    addServiceCallToActionFutureArray(metric_control_future_array, queue_it->second->expected_departure_service_, action_set_msg, ParameterType::Departure, queue_name);
                }
                else
                {
                    addServiceCallToFloatFutureArray(float_request_future_array, queue_it->second->departure_independent_from_action_service_, action_set_msg, ParameterType::Departure, queue_name);
                }
            }

            for (auto future_it = metric_control_future_array.begin(); future_it != metric_control_future_array.end(); ++future_it)
            {
                 std::shared_ptr<AsyncMetricFutureStruct<TMetricControlPredictionSrv>> future_result = future_it->get();

                if (future_result->parameter_type_ == ParameterType::Penalty)
                {
                    if (future_result->current_call_succeeded_)
                    {
                        auto& penalty_prediction_srv = future_result->current_srv_msg_;
                        if (penalty_prediction_srv.response.predictions.size() == size_of_actions)
                        {
                            for (int action_index = 0; action_index < size_of_actions; ++action_index)
                            {
                                action_parameters_output[action_index].penalty = penalty_prediction_srv.response.predictions[action_index];
                            }
                        }
                        else
                        {
                            are_parameters_valid = false;
                            ROS_WARN_STREAM_THROTTLE(2, "Returned penalty array doesn't contain the same amount of elements has the action set (expected " << size_of_actions << ", received "<< penalty_prediction_srv.response.predictions.size() <<")");
                        }
                    }
                    else
                    {
                        are_parameters_valid = false;
                        ROS_WARN_STREAM_THROTTLE(2, "Failed to call the penalty evalution service");
                    }
                }
                else if (future_result->parameter_type_ == ParameterType::RenewalTime)
                {
                    if (future_result->current_call_succeeded_)
                    {
                        auto& renewal_time_msg = future_result->current_srv_msg_;
                        if (renewal_time_msg.response.predictions.size() == size_of_actions)
                        {
                            for (int action_index = 0; action_index < size_of_actions; ++action_index)
                            {
                                action_parameters_output[action_index].expected_renewal_time = renewal_time_msg.response.predictions[action_index];
                            }
                        }
                        else
                        {
                            are_parameters_valid = false;
                            ROS_WARN_STREAM_THROTTLE(2, "Returned renewal time array doesn't contain the same amount of elements has the action set (expected " << size_of_actions << ", received "<< renewal_time_msg.response.predictions.size() <<")");
                        }
                    }
                    else
                    {
                        are_parameters_valid = false;
                        ROS_WARN_STREAM_THROTTLE(2, "Failed to call the renewal time service");
                    }
                }
                else if (future_result->parameter_type_ == ParameterType::Arrival)
                {
                    auto& arrival_predictions = future_result->current_srv_msg_;
                    string& queue_name = future_result->queue_name_;
                    if(future_result->current_call_succeeded_)
                    {
                        const int& returned_size = arrival_predictions.response.predictions.size();
                        
                        if(returned_size == size_of_actions)
                        {
                            for (int action_index =0; action_index < size_of_actions; ++action_index)
                            {
                                action_parameters_output[action_index].queue_parameters[queue_name].expected_arrivals = arrival_predictions.response.predictions[action_index];
                            }
                        }
                        else
                        {
                            are_parameters_valid = false;
                            ROS_WARN_STREAM_THROTTLE(2, "Returned prediction array doesn't contain the same amount of elements has the action set (expected " << size_of_actions << ", received "<< returned_size <<")");
                        }
                    }
                    else
                    {
                        are_parameters_valid = false;
                        ROS_WARN_STREAM_THROTTLE(2, "Failed to call the expected arrival service named: " << future_result->client_->getService());
                    }
                }
                else if(future_result->parameter_type_ == ParameterType::Departure)
                {
                    auto& departure_predictions = future_result->current_srv_msg_;
                    string& queue_name = future_result->queue_name_;
                    if(future_result->current_call_succeeded_)
                    {
                        const int& returned_size = departure_predictions.response.predictions.size();
                        
                        if(returned_size == size_of_actions)
                        {
                            for (int action_index =0; action_index < size_of_actions; ++action_index)
                            {
                                action_parameters_output[action_index].queue_parameters[queue_name].expected_departures = departure_predictions.response.predictions[action_index];
                            }
                        }
                        else
                        {
                            are_parameters_valid = false;
                            ROS_WARN_STREAM_THROTTLE(2, "Returned prediction array doesn't contain the same amount of elements has the action set (expected " << size_of_actions << ", received "<< returned_size <<")");
                        }
                    }
                    else
                    {
                        are_parameters_valid = false;
                        ROS_WARN_STREAM_THROTTLE(2, "Failed to call the expected departure service named: " << future_result->client_->getService());
                    }
                }

            }

            for (auto future_it = float_request_future_array.begin(); future_it != float_request_future_array.end(); ++future_it)
            {
                std::shared_ptr<AsyncMetricFutureStruct<ros_queue_msgs::FloatRequest>> future_result = future_it->get();
                string& queue_name = future_result->queue_name_;

                if(future_result->parameter_type_ == ParameterType::Arrival)
                {
                    if(future_result->current_call_succeeded_)
                    {
                        auto& arrival_prediction = future_result->current_srv_msg_;

                        for (int action_index =0; action_index < size_of_actions; ++action_index)
                        {
                            action_parameters_output[action_index].queue_parameters[queue_name].expected_arrivals = arrival_prediction.response.value;
                        }
                    }
                    else
                    {
                        are_parameters_valid = false;
                        ROS_WARN_STREAM_THROTTLE(2, "Failed to call the expected arrival service independent from actions named: " << future_result->client_->getService());
                    }
                }
                else if(future_result->parameter_type_ == ParameterType::Departure)
                {
                    if(future_result->current_call_succeeded_)
                    {
                        auto& departure_prediction = future_result->current_srv_msg_;
                        for (int action_index =0; action_index < size_of_actions; ++action_index)
                        {
                            action_parameters_output[action_index].queue_parameters[queue_name].expected_departures = departure_prediction.response.value;
                        }
                    }
                    else
                    {
                        are_parameters_valid = false;
                        ROS_WARN_STREAM_THROTTLE(2, "Failed to call the expected departure service independent from actions named: " << future_result->client_->getService());
                    }
                }
            }

            // Populate the output if the parameters are valid
            if (are_parameters_valid)
            {
                for (int action_index = 0; action_index < size_of_actions; ++action_index)
                {
                    for (auto queue_it = queue_list_.begin(); queue_it != queue_list_.end(); ++queue_it)
                    {
                        const string& queue_name = queue_it->first;
                        ObjectiveParameter& queue_parameters_out = action_parameters_output[action_index].queue_parameters[queue_name];

                        for (auto queue_it = server_state_msg.response.queue_server_state.queue_sizes.begin(); 
                                 queue_it != server_state_msg.response.queue_server_state.queue_sizes.end(); ++queue_it)
                        {
                            if(queue_name == queue_it->queue_name)
                            {
                                queue_parameters_out.current_size = queue_it->current_size;
                                break;
                            }
                        }
                    }
                }
            }

             return are_parameters_valid;
        }
        
        /**
         * @brief Used the min drift-plus-penalty algorithm to compute the action that minimize the penalty and stabilizes all the queues (if possible).
         * @param action_parameters_list All the parameters and variables for the penalty and the queues for each actions.
         * @return Returns the best action including all the metrics.
        */
        ActionParameters computeMinDriftPlusPenalty(std::vector<ActionParameters>& action_parameters_list)
        {
            auto best_action = action_parameters_list.begin();

            bool is_first_cost = true;
            
            TControllerCostListType action_costs_lists;

            for (auto action_parameters_it = action_parameters_list.begin(); action_parameters_it != action_parameters_list.end(); ++action_parameters_it)
            {
                ActionCostType action_named_action_costs;
                ros_queue_msgs::ControllerActionCosts action_costs_msg;
                
                const float penalty_cost = v_parameter_*action_parameters_it->penalty;
                action_parameters_it->cost = penalty_cost;
                
                if(measure_cost_)
                {                  
                    ros_queue_msgs::MetricCosts penalty_cost_msg;
                    penalty_cost_msg.metric_name = "penalty";
                    penalty_cost_msg.cost = penalty_cost;
                    action_costs_msg.metric_costs.push_back(std::move(penalty_cost_msg));
                }
                
                for(auto queue_it = action_parameters_it->queue_parameters.begin(); queue_it != action_parameters_it->queue_parameters.end(); ++queue_it)
                {
                    const string& queue_name = queue_it->first;
                    const float queue_cost = queue_list_[queue_name]->weight_ * 
                                       queue_it->second.current_size * 
                                       (queue_it->second.expected_arrivals - queue_it->second.expected_departures);
                    action_parameters_it->cost += queue_cost;

                    if(measure_cost_)
                    {
                        ros_queue_msgs::MetricCosts queue_cost_msg;
                        queue_cost_msg.metric_name = queue_name;
                        queue_cost_msg.cost = queue_cost;
                        action_costs_msg.metric_costs.push_back(std::move(queue_cost_msg));
                    }
                }

                if(measure_cost_)
                {
                    action_costs_msg.total_cost = action_parameters_it->cost;
                    action_named_action_costs.action = action_parameters_it->action;
                    action_named_action_costs.costs = action_costs_msg;
                    action_costs_lists.action_costs_list.push_back(std::move(action_named_action_costs));
                }

                if (is_first_cost)
                {
                    is_first_cost = false;
                    best_action = action_parameters_it;
                }
                else if(action_parameters_it->cost < best_action->cost)
                {
                    best_action = action_parameters_it;
                }
                else if(action_parameters_it->cost == best_action->cost)
                {
                    ROS_WARN("Queue controller: Two actions have the same cost. The first one will be selected.");
                }
            }
            
            if(measure_cost_)
            {
                cost_publisher_.publish(action_costs_lists);
            }

            return *best_action;
        }

        /**
         * @brief Used the renewal min drift-plus-penalty algorithm to compute the action that minimize the penalty and stabilizes all the queues (if possible).
         * @param action_parameters_list All the parameters and variables for the penalty and the queues for each actions.
         * @return Returns the best action including all the metrics.
        */
        ActionParameters computeRenewalMinDriftPlusPenalty(std::vector<ActionParameters>& action_parameters_list)
        {
            auto best_action = action_parameters_list.begin();

            bool is_first_cost = true;

            TControllerCostListType action_costs_lists;

            for (auto action_parameters_it = action_parameters_list.begin(); action_parameters_it != action_parameters_list.end(); ++action_parameters_it)
            {
                ActionCostType action_named_action_costs;
                ros_queue_msgs::ControllerActionCosts action_costs_msg;

                const double& expected_renewal_time = action_parameters_it->expected_renewal_time;

                float penalty_cost = v_parameter_*action_parameters_it->penalty;
                action_parameters_it->cost = penalty_cost;

                if(measure_cost_)
                {                  
                    ros_queue_msgs::MetricCosts penalty_cost_msg;
                    penalty_cost_msg.metric_name = "penalty";
                    penalty_cost_msg.cost = penalty_cost;
                    action_costs_msg.metric_costs.push_back(std::move(penalty_cost_msg));
                }

                for(auto queue_it = action_parameters_it->queue_parameters.begin(); queue_it != action_parameters_it->queue_parameters.end(); ++queue_it)
                {
                    const string& queue_name = queue_it->first;

                    // Adjusts the arrivals based on the renewal time. The values are directly changed in the object so the manual
                    // changes to the virtual queues will be based on the expected_renewal_time in the later controller steps.
                    if (queue_list_[queue_name]->is_arrival_renewal_dependent)
                    {
                        queue_it->second.expected_arrivals = expected_renewal_time*queue_it->second.expected_arrivals;
                    }
                    if (queue_list_[queue_name]->is_departure_renewal_dependent)
                    {
                        queue_it->second.expected_departures = expected_renewal_time*queue_it->second.expected_departures;
                    }

                    float queue_cost = queue_list_[queue_name]->weight_ * 
                                       queue_it->second.current_size * 
                                       (queue_it->second.expected_arrivals - queue_it->second.expected_departures);

                    action_parameters_it->cost += queue_cost;

                    if(measure_cost_)
                    {
                        ros_queue_msgs::MetricCosts queue_cost_msg;
                        queue_cost_msg.metric_name = queue_name;
                        queue_cost_msg.cost = queue_cost;
                        action_costs_msg.metric_costs.push_back(std::move(queue_cost_msg));
                    }
                }

                // TODO: Verify formulation
                if (is_penalty_renewal_dependent_ && expected_renewal_time > 0.0f)
                {
                    action_parameters_it->cost = action_parameters_it->cost/expected_renewal_time; 
                }

                if(measure_cost_)
                {
                    // Note here that the total cost is the cost per renewal time and not the sum of the costs.
                    action_costs_msg.total_cost = action_parameters_it->cost;
                    action_named_action_costs.action = action_parameters_it->action;
                    action_named_action_costs.costs = action_costs_msg;
                    action_costs_lists.action_costs_list.push_back(std::move(action_named_action_costs));
                }

                if (is_first_cost)
                {
                    is_first_cost = false;
                    best_action = action_parameters_it;
                }
                else if(action_parameters_it->cost < best_action->cost)
                {
                    best_action = action_parameters_it;
                }
            }

            if(measure_cost_)
            {
                cost_publisher_.publish(action_costs_lists);
            }

            return *best_action;
        }

        /**
         * @brief Publish the best action.
         * @param best_action The best action that was computed
        */
        void sendBestCommand(ActionType& best_action)
        {
            TActionLibOutputGoalType action_lib_output;
            action_lib_output.action_goal = best_action;

            if(best_action_client_)
            {
                best_action_client_->sendGoal(action_lib_output);
            }
        }
        
        /**
         * @brief Send a signal to the queue server to update the virtual queues based on the 
         * current state of the system.
        */
        void updateVirtualQueuesBasedOnCurrentState()
        {
            std_srvs::Empty trigger;
            if(!virtual_queues_trigger_.call(trigger))
            {
                ROS_WARN_STREAM("Failed to call the virtual queue trigger service");
            }
        }

        /**
         * @brief Send a message to the queue server to update the virual queues based on the
         * metrics of the best action
        */
        void updateVirtualQueuesBasedOnBestAction(ActionParameters& best_action_parameters)
        {
            ros_queue_msgs::VirtualQueueChangesList change_list;

            for (auto queue_it = queue_list_.begin(); queue_it != queue_list_.end(); ++queue_it)
            {
                if(queue_it->second->is_virtual_)
                {
                    const string& queue_name = queue_it->first;
                    ros_queue_msgs::VirtualQueueChanges queue_changes;
                    
                    queue_changes.queue_name = queue_name;
                    queue_changes.arrival = best_action_parameters.queue_parameters[queue_name].expected_arrivals;
                    queue_changes.departure = best_action_parameters.queue_parameters[queue_name].expected_departures;
                    
                    change_list.list.push_back(std::move(queue_changes));
                }
            }

            manual_virtual_queue_changes_.publish(change_list);
        }


        // ===== Controller parameters =====
        /**
         * @brief If set to false, the controller will find an optimal next action to take and 
         * then update the virtual queues based on that optimal solution. If set to true, 
         * the controller will update the virtual queues based on the current of state of the 
         * system then find an optimal next action. Default: set to false
        */
        bool inversed_control_and_update_steps_ = false;
        

        /**
         * @brief If set to true, the controller will trigger or send changes to the queue server base on the flag 
         * inverse_control_and_steps. If set to false, the controller will not send any changes to the specified 
         * queue_server_name. Its used if the user wants to implement another mecanism to change the virtual queues 
         * in queue server or to run multiple controller in parallel  for the same queue server where only one of 
         * them is sending changes and not the others. Default: set to true.
        */
        bool responsible_for_virtual_queue_changes_ = true;

        /**
         * @brief Indicate the controller types. It defines the type of optimization and the used metrics.
        */
        ControllerType controller_type_;

        /**
         * @brief Indicate the period between each controller's call.
         * Used by the min_drift_plus_penalty controllers.
        */
        float controller_time_step_;

        /**
         * @brief Time after which, if the actionlib output did not succeed, the controller will be called.
         * Only used by the renewal_min_drift_plus_penalty controller.
        */
        float max_renewal_time_ = 0.0f;

        /**
         * @brief Time before which if the actionlib output suceed, the controller will wait for the difference
         *  before being called. Only used by the renewal_min_drift_plus_penalty controller.
        */
        float min_renewal_time_ = 0.0f;

        /**
         * @brief ROS service client that returns a set of possible actions that the controller will used
         * to compute its objetives metrics. The best solution will be a solution in this set.
        */
        PersistentServiceClient<TPotentialActionSetSrv> solution_space_client_;
        
        /**
         * @brief Service client that evaluates the optimization variable (penalty) given the state of
         * the system and an evaluating solution;
        */
        PersistentServiceClient<TMetricControlPredictionSrv> penalty_service_client_;

        /**
         * @brief  The V parameter that is multplied with the penalty (like a weight). 
         * When a high penalty metric also increases the queue size, this parameter balances 
         * the tradeoff between a lower penalty or lower queue delays. A high V will 
         * prioritize the minimization  of the penalty while a low V will prioritize low queue 
         * sizes and delays. 
        */
        float v_parameter_;

        /**
         * @brief Indicates if the controller should optimize the time average of the ratio between the penalty and the renewal time.
         * Only used by the renewal_min_drift_plus_penalty controller.
        */
        bool is_penalty_renewal_dependent_ = false;

        /**
         * @brief Name of the queue server node linked to the queue_controller. Mainly used as a suffix
         * to solve all the ROS topic and ROS service names provided by the queue server. 
        */
        string queue_server_name_;
};
