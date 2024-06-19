#!/usr/bin/env python
import rospy

import rosbag
from datetime import datetime
import csv
import matplotlib.pyplot as plt
import matplotlib.patches as patches

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
            "/NoRew_NoInv/auv_system_node/auv_state",
            "/NoRew_NoInv/monitoring_node/localization",
            "/NoRew_NoInv/monitoring_node/real_queue",
            "/NoRew_NoInv/monitoring_node/temperature",
            "/NoRew_NoInv/monitoring_node/low_temperature",
            "/NoRew_NoInv/monitoring_node/penalty",
            "/NoRew_NoInv/queue_controller/optimization_done",
            "/NoRew_NoInv/queue_controller/control_loop_started",
            "/NoRew_NoInv/perturbation_node/action_performance"]

    def generateOutput(self, time_init, bag_name, base_init_time_on_first_value=False):
        bag = rosbag.Bag(common_experiment_utils.BAG_DIRECTORY_PATH + bag_name + ".bag")

        # Get init time if not set
        init_time_set = False

        for topic, msg, t in bag.read_messages(topics=["/NoRew_NoInv/queue_server/server_stats"]):
            if (not init_time_set) and base_init_time_on_first_value:
                time_init = t.to_sec()
                init_time_set = True
                break

        # Get action performances
        action_performances = common_experiment_utils.ActionSeries()
        action_performances.populateWithBag(bag, "/NoRew_NoInv/", time_init)

        # Get queue server end values
        queue_server_arrival_departures_end_values = common_experiment_utils.QueueEndValues()
        
        for topic, msg, t in bag.read_messages(topics=["/NoRew_NoInv/monitoring_node/penalty"]):
            queue_server_arrival_departures_end_values.penalty.values.append(msg.queue_server_time_average_value)

        queue_server_arrival_departures_end_values.localization_arrival.values.append(action_performances.synchronized_queue_stats.localization_stats.time_average_arrival.values[-1])
        queue_server_arrival_departures_end_values.localization_departure.values.append(action_performances.synchronized_queue_stats.localization_stats.time_average_departure.values[-1])
        queue_server_arrival_departures_end_values.temperature_arrival.values.append(action_performances.synchronized_queue_stats.temperature_stats.time_average_arrival.values[-1])
        queue_server_arrival_departures_end_values.temperature_departure.values.append(action_performances.synchronized_queue_stats.temperature_stats.time_average_departure.values[-1])
        queue_server_arrival_departures_end_values.low_temperature_arrival.values.append(action_performances.synchronized_queue_stats.low_temperature_stats.time_average_arrival.values[-1])
        queue_server_arrival_departures_end_values.low_temperature_departure.values.append(action_performances.synchronized_queue_stats.low_temperature_stats.time_average_departure.values[-1])
        queue_server_arrival_departures_end_values.real_queue_arrival.values.append(action_performances.synchronized_queue_stats.real_queue_stats.time_average_arrival.values[-1])
        queue_server_arrival_departures_end_values.real_queue_departure.values.append(action_performances.synchronized_queue_stats.real_queue_stats.time_average_departure.values[-1])
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
                            action_performances.time_stamps,
                            action_performances.synchronized_queue_stats.localization_stats.queue_size,
                            action_performances.synchronized_queue_stats.temperature_stats.queue_size,
                            action_performances.synchronized_queue_stats.low_temperature_stats.queue_size,
                            action_performances.synchronized_queue_stats.real_queue_stats.queue_size,
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

        # ==== Create plots ====
        fig1, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)
        ax1.set_title("Average of the metrics of the simulated system over time under a min drift-plus-penalty queue controller in ROS")
        ax1.plot(all_metric_performance_structs.localization.time_stamps.values, all_metric_performance_structs.localization.real_continous_average_value.values, label="Metric")
        ax1.plot(all_metric_performance_structs.localization.time_stamps.values, all_metric_performance_structs.localization.target_value.values,label="Target", ls="--")
        ax1.set_ylabel("Localization uncertainty (m)")
        ax1.grid(False)
        ax1.legend()
        
        ax2.plot(all_metric_performance_structs.temperature.time_stamps.values, all_metric_performance_structs.temperature.real_continous_average_value.values, label="Metric")
        ax2.plot(all_metric_performance_structs.temperature.time_stamps.values, all_metric_performance_structs.temperature.target_value.values, label="Upper target", ls="--")
        ax2.plot(all_metric_performance_structs.low_temperature.time_stamps.values, all_metric_performance_structs.low_temperature.target_value.values, label="Lower Target", ls="--")
        ax2.grid(False)
        ax2.set_ylabel("Temperature (°C)")
        ax2.legend()

        ax3.plot(all_metric_performance_structs.real_queue.time_stamps.values, all_metric_performance_structs.real_queue.queue_server_arrival_mean.values, label="Arrival task rate mean")
        ax3.plot(all_metric_performance_structs.real_queue.time_stamps.values, all_metric_performance_structs.real_queue.target_value.values, label="Departure task rate mean")
        ax3.grid(False)
        ax3.set_ylabel("Task rate (Task/s)")
        ax3.legend()

        ax4.plot(all_metric_performance_structs.penalty.time_stamps.values, all_metric_performance_structs.penalty.real_time_average_value.values, label="Energy usage (Penalty) (J)")
        ax4.set_ylabel("Penalty (J)")
        ax4.grid(False)
        ax4.set_xlabel("Time (s)")

        ## Queue size graph and action sacrifices
        time_frame = [57.4, 58.1]
        #time_frame = [0, 60]
        fig2, (ax5) = plt.subplots(1, 1, sharex=True)
        ax5.set_title("Queue sizes balancing over time under a min drift-plus-penalty queue controller in ROS")
        ax5.plot(action_performances.time_stamps.values, action_performances.synchronized_queue_stats.localization_stats.queue_size.values, label="Localization queue", drawstyle='steps-post')
        ax5.plot(action_performances.time_stamps.values, action_performances.synchronized_queue_stats.temperature_stats.queue_size.values, label="Temperature queue", drawstyle='steps-post')
        ax5.plot(action_performances.time_stamps.values, action_performances.synchronized_queue_stats.low_temperature_stats.queue_size.values, label="Low temperature queue", drawstyle='steps-post')
        ax5.plot(action_performances.time_stamps.values, action_performances.synchronized_queue_stats.real_queue_stats.queue_size.values, label="Task queue", drawstyle='steps-post')
        ax5.set_xlim(time_frame[0], time_frame[1])
        ax5.set_ylabel("Queue sizes")
        ax5.legend()
        
        # Add actions
        max_queue_size = max(action_performances.synchronized_queue_stats.localization_stats.queue_size.values + action_performances.synchronized_queue_stats.temperature_stats.queue_size.values + action_performances.synchronized_queue_stats.low_temperature_stats.queue_size.values + action_performances.synchronized_queue_stats.real_queue_stats.queue_size.values)
        time_frame_start_index = 0
        time_frame_end_index = 0
        start_found = False
        for action_index in range(len(action_performances.time_stamps.values)):
            if action_performances.time_stamps.values[action_index] >= time_frame[0] and not start_found:
                time_frame_start_index = action_index
                start_found = True
            if action_performances.time_stamps.values[action_index] >= time_frame[1]:
                time_frame_end_index = action_index
                break
        
        action_color_dict = {common_experiment_utils.ActionType.TASK_ZONE: "#00CAFF", 
                      common_experiment_utils.ActionType.HIGH_LOCALIZATION_ZONE: "#FF00CA", 
                      common_experiment_utils.ActionType.LOW_TEMPERATURE_ZONE: "#CAFF00"}
        action_label_dict = {common_experiment_utils.ActionType.TASK_ZONE: "TaskZ",
                        common_experiment_utils.ActionType.HIGH_LOCALIZATION_ZONE: "LocZ",
                        common_experiment_utils.ActionType.LOW_TEMPERATURE_ZONE: "ColdZ"}

        last_action_index = time_frame_start_index
        for action_index in range(time_frame_start_index, time_frame_end_index):
            if action_performances.applied_action.values[last_action_index] != action_performances.applied_action.values[action_index]:
                last_action = action_performances.applied_action.values[last_action_index]
                start = action_performances.time_stamps.values[last_action_index]
                elapsed_time = action_performances.time_stamps.values[action_index] - start
                rect = patches.Rectangle((start, 0), elapsed_time, max_queue_size, color=action_color_dict[last_action], alpha=0.25)
                ax5.add_patch(rect)
                ax5.text(start + elapsed_time/2, max_queue_size-1.5, action_label_dict[last_action], ha='center', va='center')
                last_action_index = action_index

        plt.show()

    def prefixToBagName(self, prefix):
        self.bag_name = prefix + self.bag_name

    def getBagName(self):
        return self.bag_name
    
    def getROSBagArguments(self):
        topic_args = ""
        for topic in self.topics_to_record:
            topic_args += " " + topic
        
        return "-O "+ common_experiment_utils.BAG_DIRECTORY_PATH + self.bag_name + ".bag" + topic_args

