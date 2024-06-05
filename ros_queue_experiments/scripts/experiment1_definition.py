#!/usr/bin/env python
import rospy

import rosbag
from datetime import datetime
import csv
from ros_queue_msgs.msg import QueueServerStats

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
            "/NoRew_NoInv/monitoring_node/temperature"]

    def generateOutput(self, time_init):
        bag = rosbag.Bag(common_experiment_utils.BAG_DIRECTORY_PATH + self.bag_name + ".bag")
        
        queue_server_stats = common_experiment_utils.QueueServerStatsStruct()
        for topic, msg, t in bag.read_messages(topics=["/NoRew_NoInv/queue_server/server_stats"]):
            queue_server_stats.time_stamps.append((t.to_sec()-time_init))
            
            for queue_stats in msg.queue_stats:
                if queue_stats.queue_name == "LocalizationQueue":
                    queue_server_stats.localization_stats.queue_size.append(queue_stats.current_size)
                    queue_server_stats.localization_stats.time_average_arrival.append(queue_stats.arrival_time_average)
                    queue_server_stats.localization_stats.time_average_departure.append(queue_stats.departure_time_average)
                elif queue_stats.queue_name == "TemperatureQueue":
                    queue_server_stats.temperture_stats.queue_size.append(queue_stats.current_size)
                    queue_server_stats.temperture_stats.time_average_arrival.append(queue_stats.arrival_time_average)
                    queue_server_stats.temperture_stats.time_average_departure.append(queue_stats.departure_time_average)
                elif queue_stats.queue_name == "LowTemperatureQueue":
                    queue_server_stats.low_temperature_stats.queue_size.append(queue_stats.current_size)
                    queue_server_stats.low_temperature_stats.time_average_arrival.append(queue_stats.arrival_time_average)
                    queue_server_stats.low_temperature_stats.time_average_departure.append(queue_stats.departure_time_average)
                elif queue_stats.queue_name == "TaskQueue":
                    queue_server_stats.real_queue_stats.queue_size.append(queue_stats.current_size)
                    queue_server_stats.real_queue_stats.time_average_arrival.append(queue_stats.arrival_time_average)
                    queue_server_stats.real_queue_stats.time_average_departure.append(queue_stats.departure_time_average)
        
        
        rows = []
        for i in range(len(queue_server_stats.time_stamps)):
            row = []
            row.append(queue_server_stats.time_stamps[i])
            row.append(queue_server_stats.localization_stats.time_average_arrival[i])
            row.append(queue_server_stats.localization_stats.time_average_departure[i])
            row.append(queue_server_stats.temperture_stats.time_average_arrival[i])
            row.append(queue_server_stats.temperture_stats.time_average_departure[i])
            row.append(queue_server_stats.low_temperature_stats.time_average_arrival[i])
            row.append(queue_server_stats.low_temperature_stats.time_average_departure[i])
            row.append(queue_server_stats.real_queue_stats.time_average_arrival[i])
            row.append(queue_server_stats.real_queue_stats.time_average_departure[i])
            rows.append(row)
        csv_filename = common_experiment_utils.RESULT_DIRECTORY_PATH + self.bag_name + ".csv"
        with open(csv_filename, 'w') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=';')
            csvwriter.writerow(["time", "localization_arrival", "localization_departure", "temperature_arrival", "temperature_departure", "low_temperature_arrival", "low_temperature_departure", "real_queue_arrival", "real_queue_departure"])
            csvwriter.writerows(rows)

    def prefixToBagName(self, prefix):
        self.bag_name = prefix + self.bag_name

    def getBagName(self):
        return self.bag_name
    
    def getROSBagArguments(self):
        topic_args = ""
        for topic in self.topics_to_record:
            topic_args += " " + topic
        
        return "-O "+ common_experiment_utils.BAG_DIRECTORY_PATH + self.bag_name + ".bag" + topic_args

