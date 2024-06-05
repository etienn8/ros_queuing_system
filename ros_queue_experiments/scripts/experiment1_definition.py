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
            "/NoRew_NoInv/monitoring_node/penalty"]

    def generateOutput(self, time_init):
        bag = rosbag.Bag(common_experiment_utils.BAG_DIRECTORY_PATH + self.bag_name + ".bag")


        # Get queue server end values
        queue_server_stats = common_experiment_utils.QueueServerStatsStruct()
        queue_server_arrival_departures_end_values = common_experiment_utils.QueueEndValues()

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
         
        penalty_list = []
        for topic, msg, t in bag.read_messages(topics=["/NoRew_NoInv/monitoring_node/penalty"]):
            penalty_list.append(msg.queue_server_time_average_value)

        queue_server_arrival_departures_end_values.localization_arrival = queue_server_stats.localization_stats.time_average_arrival[-1]
        queue_server_arrival_departures_end_values.localization_departure = queue_server_stats.localization_stats.time_average_departure[-1]
        queue_server_arrival_departures_end_values.temperature_arrival = queue_server_stats.temperture_stats.time_average_arrival[-1]
        queue_server_arrival_departures_end_values.temperature_departure = queue_server_stats.temperture_stats.time_average_departure[-1]
        queue_server_arrival_departures_end_values.low_temperature_arrival = queue_server_stats.low_temperature_stats.time_average_arrival[-1]
        queue_server_arrival_departures_end_values.low_temperature_departure = queue_server_stats.low_temperature_stats.time_average_departure[-1]
        queue_server_arrival_departures_end_values.real_queue_arrival = queue_server_stats.real_queue_stats.time_average_arrival[-1]
        queue_server_arrival_departures_end_values.real_queue_departure = queue_server_stats.real_queue_stats.time_average_departure[-1]
        queue_server_arrival_departures_end_values.penalty = penalty_list[-1]
        
        # Get performance metrics 
        all_metric_performance_structs = common_experiment_utils.AllMetricPerformanceStruct()
        all_metric_performance_structs.populateWithBag(bag, "/NoRew_NoInv/", time_init)

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

            if (i == 0):
                row.append(queue_server_arrival_departures_end_values.localization_arrival)
                row.append(queue_server_arrival_departures_end_values.localization_departure)
                row.append(queue_server_arrival_departures_end_values.temperature_arrival)
                row.append(queue_server_arrival_departures_end_values.temperature_departure)
                row.append(queue_server_arrival_departures_end_values.low_temperature_arrival)
                row.append(queue_server_arrival_departures_end_values.low_temperature_departure)
                row.append(queue_server_arrival_departures_end_values.real_queue_arrival)
                row.append(queue_server_arrival_departures_end_values.real_queue_departure)
                row.append(queue_server_arrival_departures_end_values.penalty)
            else:
                for j in range(9):
                    row.append("")

            rows.append(row)
        
        csv_filename = common_experiment_utils.RESULT_DIRECTORY_PATH + self.bag_name + ".csv"
        
        with open(csv_filename, 'w') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=';')
            csvwriter.writerow(["time", 
                               "localization_arrival", 
                               "localization_departure", 
                               "temperature_arrival", 
                               "temperature_departure", 
                               "low_temperature_arrival", 
                                "low_temperature_departure",
                                "real_queue_arrival", 
                                "real_queue_departure",
                                "last_localization_arrival",
                                "last_localization_departure",
                                "last_temperature_arrival",
                                "last_temperature_departure",
                                "last_low_temperature_arrival",
                                "last_low_temperature_departure",
                                "last_real_queue_arrival",
                                "last_real_queue_departure",
                                "penalty"])
            
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

