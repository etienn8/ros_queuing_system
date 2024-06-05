#!/usr/bin/env python
import os
import rosbag
import rospy
import csv

from ros_queue_experiments.msg import MetricPerformance

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
        self.queue_size = []
        self.time_average_arrival = []
        self.time_average_departure = []

class QueueServerStatsStruct:
    def __init__(self):
        self.time_stamps = []
        self.localization_stats = QueueServerMetricStatsStruct()
        self.temperture_stats = QueueServerMetricStatsStruct()
        self.low_temperature_stats = QueueServerMetricStatsStruct()
        self.real_queue_stats = QueueServerMetricStatsStruct()

class MetricPerformanceStruct:
    def __init__(self):
        self.time_stamps = []

        self.current_real_value = []
        self.real_average_value = []
        self.real_continous_average_value = []
        self.real_time_average_value = []
        self.queue_server_time_average_value = []
        self.queue_server_arrival_mean = []
        self.target_value = []
        self.real_current_diff_with_target = []
        self.real_continuous_average_diff_with_target = []
        self.real_continuous_average_diff_with_server_mean = []
        self.real_continuous_average_diff_with_server_time_average = []


class AllMetricPerformanceStruct:
    def __init__(self):
        self.localization = MetricPerformanceStruct()
        self.temperature = MetricPerformanceStruct()
        self.real_queue = MetricPerformanceStruct()
        self.penalty = MetricPerformanceStruct()

    def populateWithBag(self, bag: rosbag.Bag, monitoring_prefix: str, time_init: rospy.Time):
        self.__populateMetric(bag, monitoring_prefix + "monitoring_node/localization", self.localization, time_init)
        self.__populateMetric(bag, monitoring_prefix + "monitoring_node/temperature", self.temperature, time_init)
        self.__populateMetric(bag, monitoring_prefix + "monitoring_node/real_queue", self.real_queue, time_init)
        self.__populateMetric(bag, monitoring_prefix + "monitoring_node/penalty", self.penalty, time_init)
 
    def __populateMetric(self, bag: rosbag.Bag, topic_name: str, metric: MetricPerformanceStruct, time_init: float):
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            metric.time_stamps.append(t.to_sec() - time_init)
            metric.current_real_value.append(msg.current_real_value)
            metric.real_average_value.append(msg.real_average_value)
            metric.real_continous_average_value.append(msg.real_continous_average_value)
            metric.real_time_average_value.append(msg.real_time_average_value)
            metric.queue_server_time_average_value.append(msg.queue_server_time_average_value)
            metric.queue_server_arrival_mean.append(msg.queue_server_arrival_mean)
            metric.target_value.append(msg.target_value)
            metric.real_current_diff_with_target.append(msg.real_current_diff_with_target)
            metric.real_continuous_average_diff_with_target.append(msg.real_continuous_average_diff_with_target)
            metric.real_continuous_average_diff_with_server_mean.append(msg.real_continuous_average_diff_with_server_mean)
            metric.real_continuous_average_diff_with_server_time_average.append(msg.real_continuous_average_diff_with_server_time_average)

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


class QueueEndValues:
    def __init__(self):
        self.localization_arrival = 0.0
        self.localization_departure = 0.0

        self.temperature_arrival = 0.0
        self.temperature_departure = 0.0

        self.low_temperature_arrival = 0.0
        self.low_temperature_departure = 0.0

        self.real_queue_arrival = 0.0
        self.real_queue_departure = 0.0

        self.penalty = 0.0
