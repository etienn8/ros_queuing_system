#!/usr/bin/env python
import rospy

import rosbag
from datetime import datetime
import csv
from ros_queue_msgs.msg import QueueServerStats
from ros_queue_experiments.msg import MetricPerformance

import common_experiment_utils

class Experiment1Analyser:
    def __init__(self):
        time_now = datetime.now()
        string_time_now = time_now.strftime("%Y-%m-%d_%H-%M-%S")
        self.bag_name = "experiment1_" + string_time_now

        self.topics_to_record = [
            "/NoRew_NoInv/queue_server/server_stats",
            "/NoRew_NoInv/monitoring_node/synced_action_performances",
            "/NoRew_NoInv/auv_system_node/auv_state",
            "/NoRew_NoInv/monitoring_node/localization",
            "/NoRew_NoInv/monitoring_node/real_queue",
            "/NoRew_NoInv/monitoring_node/temperature",
             "/NoRew_NoInv/monitoring_node/low_temperature",
            "/NoRew_NoInv/monitoring_node/penalty"]

    def generateOutput(self, time_init):
        bag = rosbag.Bag(common_experiment_utils.BAG_DIRECTORY_PATH + self.bag_name + ".bag")


        # Get queue server end values
        queue_server_stats = common_experiment_utils.QueueServerStatsStruct()
        queue_server_arrival_departures_end_values = common_experiment_utils.QueueEndValues()

        for topic, msg, t in bag.read_messages(topics=["/NoRew_NoInv/queue_server/server_stats"]):
            queue_server_stats.time_stamps.values.append((t.to_sec()-time_init))
            
            for queue_stats in msg.queue_stats:
                if queue_stats.queue_name == "LocalizationQueue":
                    queue_server_stats.localization_stats.queue_size.values.append(queue_stats.current_size)
                    queue_server_stats.localization_stats.time_average_arrival.values.append(queue_stats.arrival_time_average)
                    queue_server_stats.localization_stats.time_average_departure.values.append(queue_stats.departure_time_average)
                elif queue_stats.queue_name == "TemperatureQueue":
                    queue_server_stats.temperature_stats.queue_size.values.append(queue_stats.current_size)
                    queue_server_stats.temperature_stats.time_average_arrival.values.append(queue_stats.arrival_time_average)
                    queue_server_stats.temperature_stats.time_average_departure.values.append(queue_stats.departure_time_average)
                elif queue_stats.queue_name == "LowTemperatureQueue":
                    queue_server_stats.low_temperature_stats.queue_size.values.append(queue_stats.current_size)
                    queue_server_stats.low_temperature_stats.time_average_arrival.values.append(queue_stats.arrival_time_average)
                    queue_server_stats.low_temperature_stats.time_average_departure.values.append(queue_stats.departure_time_average)
                elif queue_stats.queue_name == "TaskQueue":
                    queue_server_stats.real_queue_stats.queue_size.values.append(queue_stats.current_size)
                    queue_server_stats.real_queue_stats.time_average_arrival.values.append(queue_stats.arrival_time_average)
                    queue_server_stats.real_queue_stats.time_average_departure.values.append(queue_stats.departure_time_average)
         
        
        for topic, msg, t in bag.read_messages(topics=["/NoRew_NoInv/monitoring_node/penalty"]):
            queue_server_arrival_departures_end_values.penalty.values.append(msg.queue_server_time_average_value)

        queue_server_arrival_departures_end_values.localization_arrival.values.append(queue_server_stats.localization_stats.time_average_arrival.values[-1])
        queue_server_arrival_departures_end_values.localization_departure.values.append(queue_server_stats.localization_stats.time_average_departure.values[-1])
        queue_server_arrival_departures_end_values.temperature_arrival.values.append(queue_server_stats.temperature_stats.time_average_arrival.values[-1])
        queue_server_arrival_departures_end_values.temperature_departure.values.append(queue_server_stats.temperature_stats.time_average_departure.values[-1])
        queue_server_arrival_departures_end_values.low_temperature_arrival.values.append(queue_server_stats.low_temperature_stats.time_average_arrival.values[-1])
        queue_server_arrival_departures_end_values.low_temperature_departure.values.append(queue_server_stats.low_temperature_stats.time_average_departure.values[-1])
        queue_server_arrival_departures_end_values.real_queue_arrival.values.append(queue_server_stats.real_queue_stats.time_average_arrival.values[-1])
        queue_server_arrival_departures_end_values.real_queue_departure.values.append(queue_server_stats.real_queue_stats.time_average_departure.values[-1])
        queue_server_arrival_departures_end_values.penalty.values = [queue_server_arrival_departures_end_values.penalty.values[-1]]

        # Get performance metrics 
        all_metric_performance_structs = common_experiment_utils.AllMetricPerformanceStruct()
        all_metric_performance_structs.populateWithBag(bag, "/NoRew_NoInv/", time_init)
        
        # Create output CSV
        separator_second_graph = common_experiment_utils.Series()
        separator_second_graph.variable_name = "controller_sacrifices_one_queue"
        separator_end_values_table = common_experiment_utils.Series()
        separator_end_values_table.variable_name = "end_values_table"

        csv_filename = common_experiment_utils.RESULT_DIRECTORY_PATH + self.bag_name + ".csv"
        series_to_record = [all_metric_performance_structs.localization.time_stamps,
                            all_metric_performance_structs.localization.real_continous_average_value,
                            all_metric_performance_structs.localization.target_value,
                            all_metric_performance_structs.temperature.time_stamps,
                            all_metric_performance_structs.temperature.real_continous_average_value,
                            all_metric_performance_structs.temperature.target_value,
                            all_metric_performance_structs.low_temperature.target_value,
                            all_metric_performance_structs.real_queue.time_stamps,
                            all_metric_performance_structs.real_queue.queue_server_arrival_mean,
                            all_metric_performance_structs.real_queue.target_value,
                            all_metric_performance_structs.penalty.time_stamps,
                            all_metric_performance_structs.penalty.real_time_average_value,
                            separator_second_graph,
                            queue_server_stats.time_stamps,
                            queue_server_stats.localization_stats.queue_size,
                            queue_server_stats.temperature_stats.queue_size,
                            queue_server_stats.low_temperature_stats.queue_size,
                            queue_server_stats.real_queue_stats.queue_size,
                            separator_end_values_table,
                            queue_server_arrival_departures_end_values.localization_arrival,
                            queue_server_arrival_departures_end_values.localization_departure,
                            queue_server_arrival_departures_end_values.temperature_arrival,
                            queue_server_arrival_departures_end_values.temperature_departure,
                            queue_server_arrival_departures_end_values.low_temperature_arrival,
                            queue_server_arrival_departures_end_values.low_temperature_departure,
                            queue_server_arrival_departures_end_values.real_queue_arrival,
                            queue_server_arrival_departures_end_values.real_queue_departure,
                            queue_server_arrival_departures_end_values.penalty]
        
        common_experiment_utils.createCSV(series_to_record, csv_filename)




    def prefixToBagName(self, prefix):
        self.bag_name = prefix + self.bag_name

    def getBagName(self):
        return self.bag_name
    
    def getROSBagArguments(self):
        topic_args = ""
        for topic in self.topics_to_record:
            topic_args += " " + topic
        
        return "-O "+ common_experiment_utils.BAG_DIRECTORY_PATH + self.bag_name + ".bag" + topic_args

