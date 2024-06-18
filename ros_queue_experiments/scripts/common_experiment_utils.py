#!/usr/bin/env python
import os
import rosbag
import rospy
import csv
from enum import Enum

from ros_queue_experiments.msg import MetricPerformance
from ros_queue_experiments.msg import ActionPerformance

script_path_list = os.path.normpath(os.path.abspath(__file__)).split(os.sep)
package_path = script_path_list[:-2]
LAUNCH_DIRECTORY_PATH = os.sep.join(package_path) + "/launch/"
BAG_DIRECTORY_PATH = os.sep.join(package_path) + "/experiment_bags/"
RESULT_DIRECTORY_PATH = BAG_DIRECTORY_PATH + "results/"

class Series:
    def __init__(self):
        self.variable_name = ""
        self.values = []

class QueueServerMetricStatsStruct:
    def __init__(self):
        self.queue_size = Series()
        self.time_average_arrival = Series()
        self.time_average_departure = Series()

class QueueServerStatsStruct:
    def __init__(self):
        self.time_stamps = Series()
        self.time_stamps.variable_name = "queue_server_time"

        self.localization_stats = QueueServerMetricStatsStruct()
        self.localization_stats.queue_size.variable_name = "localization_queue_size"
        self.localization_stats.time_average_arrival.variable_name = "localization_time_average_arrival"
        self.localization_stats.time_average_departure.variable_name = "localization_time_average_departure"

        self.temperature_stats = QueueServerMetricStatsStruct()
        self.temperature_stats.queue_size.variable_name = "temperature_queue_size"
        self.temperature_stats.time_average_arrival.variable_name = "temperature_time_average_arrival"
        self.temperature_stats.time_average_departure.variable_name = "temperature_time_average_departure"

        self.low_temperature_stats = QueueServerMetricStatsStruct()
        self.low_temperature_stats.queue_size.variable_name = "low_temperature_queue_size"
        self.low_temperature_stats.time_average_arrival.variable_name = "low_temperature_time_average_arrival"
        self.low_temperature_stats.time_average_departure.variable_name = "low_temperature_time_average_departure"

        self.real_queue_stats = QueueServerMetricStatsStruct()
        self.real_queue_stats.queue_size.variable_name = "real_queue_size"
        self.real_queue_stats.time_average_arrival.variable_name = "real_queue_time_average_arrival"
        self.real_queue_stats.time_average_departure.variable_name = "real_queue_time_average_departure"

class MetricPerformanceStruct:
    def __init__(self, metric_name: str = ""):
        self.time_stamps = Series()
        self.time_stamps.variable_name = metric_name + "_time"
        
        self.current_real_value = Series()
        self.current_real_value.variable_name = metric_name + "_current_real_value"

        self.real_average_value = Series()
        self.real_average_value.variable_name = metric_name + "_real_average_value"

        self.real_continous_average_value = Series()
        self.real_continous_average_value.variable_name = metric_name + "_real_continous_average_value"

        self.real_time_average_value = Series()
        self.real_time_average_value.variable_name = metric_name + "_real_time_average_value"

        self.queue_server_time_average_value = Series()
        self.queue_server_time_average_value.variable_name = metric_name + "_queue_server_time_average_value"

        self.queue_server_arrival_mean = Series()
        self.queue_server_arrival_mean.variable_name = metric_name + "_queue_server_arrival_mean"

        self.target_value = Series()
        self.target_value.variable_name = metric_name + "_target_value"

        self.real_current_diff_with_target = Series()
        self.real_current_diff_with_target.variable_name = metric_name + "_real_current_diff_with_target"

        self.real_continuous_average_diff_with_target = Series()
        self.real_continuous_average_diff_with_target.variable_name = metric_name + "_real_continuous_average_diff_with_target"

        self.real_continuous_average_diff_with_server_mean = Series()
        self.real_continuous_average_diff_with_server_mean.variable_name = metric_name + "_real_continuous_average_diff_with_server_mean"

        self.real_continuous_average_diff_with_server_time_average = Series()
        self.real_continuous_average_diff_with_server_time_average.variable_name = metric_name + "_real_continuous_average_diff_with_server_time_average"

class AllMetricPerformanceStruct:
    def __init__(self):
        self.localization = MetricPerformanceStruct("localization")
        self.temperature = MetricPerformanceStruct("temperature")
        self.low_temperature = MetricPerformanceStruct("low_temperature")
        self.real_queue = MetricPerformanceStruct("real_queue")
        self.penalty = MetricPerformanceStruct("penalty")

    def populateWithBag(self, bag: rosbag.Bag, monitoring_prefix: str, time_init: rospy.Time):
        self.__populateMetric(bag, monitoring_prefix + "monitoring_node/localization", self.localization, time_init)
        self.__populateMetric(bag, monitoring_prefix + "monitoring_node/temperature", self.temperature, time_init)
        self.__populateMetric(bag, monitoring_prefix + "monitoring_node/low_temperature", self.low_temperature, time_init)
        self.__populateMetric(bag, monitoring_prefix + "monitoring_node/real_queue", self.real_queue, time_init)
        self.__populateMetric(bag, monitoring_prefix + "monitoring_node/penalty", self.penalty, time_init)
 
    def __populateMetric(self, bag: rosbag.Bag, topic_name: str, metric: MetricPerformanceStruct, time_init: float):
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            metric.time_stamps.values.append(t.to_sec() - time_init)
            metric.current_real_value.values.append(msg.current_real_value)
            metric.real_average_value.values.append(msg.real_average_value)
            metric.real_continous_average_value.values.append(msg.real_continous_average_value)
            metric.real_time_average_value.values.append(msg.real_time_average_value)
            metric.queue_server_time_average_value.values.append(msg.queue_server_time_average_value)
            metric.queue_server_arrival_mean.values.append(msg.queue_server_arrival_mean)
            metric.target_value.values.append(msg.target_value)
            metric.real_current_diff_with_target.values.append(msg.real_current_diff_with_target)
            metric.real_continuous_average_diff_with_target.values.append(msg.real_continuous_average_diff_with_target)
            metric.real_continuous_average_diff_with_server_mean.values.append(msg.real_continuous_average_diff_with_server_mean)
            metric.real_continuous_average_diff_with_server_time_average.values.append(msg.real_continuous_average_diff_with_server_time_average)

def createCSV(list_of_series, csv_filename):
    max_nb_rows = max([len(series.values) for series in list_of_series])
    
    with open(csv_filename, 'w') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=';')
        
        row = []
        for series in list_of_series:
            row.append(series.variable_name)

        csvwriter.writerow(row)
        for row_index in range(max_nb_rows):
            row = []
            for series in list_of_series:
                if row_index < len(series.values):
                    row.append(series.values[row_index])
                else:
                    row.append("")
            csvwriter.writerow(row)

class ActionType(Enum):
    TASK_ZONE = 0,
    HIGH_LOCALIZATION_ZONE = 1,
    LOW_TEMPERATURE_ZONE = 2


class ActionSeries:
    def __init__(self):
        self.time_stamps = Series()
        self.time_stamps.variable_name = "action_time"

        self.selected_action = Series()
        self.selected_action.variable_name = "selected_action"

        self.applied_action = Series()
        self.applied_action.variable_name = "applied_action"

        self.synchronized_queue_stats = QueueServerStatsStruct()
    
    def populateWithBag(self, bag: rosbag.Bag, monitoring_prefix: str, time_init: rospy.Time):
        for topic, msg, t in bag.read_messages(topics=[monitoring_prefix + "perturbation_node/action_performance"]):
            self.time_stamps.values.append(t.to_sec() - time_init)
            
            for internal_struct,msg_action in zip([self.selected_action, self.applied_action], [msg.controller_action_index, msg.applied_action_index]):
                if msg_action == 0:
                    internal_struct.values.append(ActionType.TASK_ZONE)
                elif msg_action == 1:
                    internal_struct.values.append(ActionType.HIGH_LOCALIZATION_ZONE)
                elif msg_action == 2:
                    internal_struct.values.append(ActionType.LOW_TEMPERATURE_ZONE)
            
            for queue_stats in msg.queue_server_stats.queue_stats:
                if queue_stats.queue_name == "LocalizationQueue":
                    self.synchronized_queue_stats.localization_stats.queue_size.values.append(queue_stats.current_size)
                    self.synchronized_queue_stats.localization_stats.time_average_arrival.values.append(queue_stats.arrival_time_average)
                    self.synchronized_queue_stats.localization_stats.time_average_departure.values.append(queue_stats.departure_time_average)
                elif queue_stats.queue_name == "TemperatureQueue":
                    self.synchronized_queue_stats.temperature_stats.queue_size.values.append(queue_stats.current_size)
                    self.synchronized_queue_stats.temperature_stats.time_average_arrival.values.append(queue_stats.arrival_time_average)
                    self.synchronized_queue_stats.temperature_stats.time_average_departure.values.append(queue_stats.departure_time_average)
                elif queue_stats.queue_name == "LowTemperatureQueue":
                    self.synchronized_queue_stats.low_temperature_stats.queue_size.values.append(queue_stats.current_size)
                    self.synchronized_queue_stats.low_temperature_stats.time_average_arrival.values.append(queue_stats.arrival_time_average)
                    self.synchronized_queue_stats.low_temperature_stats.time_average_departure.values.append(queue_stats.departure_time_average)
                elif queue_stats.queue_name == "TaskQueue":
                    self.synchronized_queue_stats.real_queue_stats.queue_size.values.append(queue_stats.current_size)
                    self.synchronized_queue_stats.real_queue_stats.time_average_arrival.values.append(queue_stats.arrival_time_average)
                    self.synchronized_queue_stats.real_queue_stats.time_average_departure.values.append(queue_stats.departure_time_average)


class QueueEndValues:
    def __init__(self):
        self.localization_arrival = Series()
        self.localization_arrival.variable_name = "end_localization_arrival"

        self.localization_departure = Series()
        self.localization_departure.variable_name = "end_localization_departure"

        self.temperature_arrival = Series()
        self.temperature_arrival.variable_name = "end_temperature_arrival"

        self.temperature_departure = Series()
        self.temperature_departure.variable_name = "end_temperature_departure"

        self.low_temperature_arrival = Series()
        self.low_temperature_arrival.variable_name = "end_low_temperature_arrival"

        self.low_temperature_departure = Series()
        self.low_temperature_departure.variable_name = "end_low_temperature_departure"

        self.real_queue_arrival = Series()
        self.real_queue_arrival.variable_name = "end_real_queue_arrival"

        self.real_queue_departure = Series()
        self.real_queue_departure.variable_name = "end_real_queue_departure"

        self.penalty = Series()
        self.penalty.variable_name = "end_penalty"
