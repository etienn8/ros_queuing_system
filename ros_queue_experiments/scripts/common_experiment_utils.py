#!/usr/bin/env python
import os

script_path_list = os.path.normpath(os.path.abspath(__file__)).split(os.sep)
package_path = script_path_list[:-2]
LAUNCH_DIRECTORY_PATH = os.sep.join(package_path) + "/launch/"
BAG_DIRECTORY_PATH = os.sep.join(package_path) + "/experiment_bags/"
RESULT_DIRECTORY_PATH = BAG_DIRECTORY_PATH + "results/"

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

