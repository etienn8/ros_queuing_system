#!/usr/bin/env python
import rospy

import rosbag
from datetime import datetime
import csv
import matplotlib.pyplot as plt

from ros_queue_msgs.msg import QueueServerStats
from ros_queue_experiments.msg import MetricPerformance

import common_experiment_utils

class Experiment2Analyser:
    def __init__(self):
        time_now = datetime.now()
        self.string_time_now = time_now.strftime("%Y-%m-%d_%H-%M-%S")
        self.normal_bag_name = "normal_experiment2_" + self.string_time_now
        self.perturbation_bag_name = "perturbation_experiment2_" + self.string_time_now

        self.topics_to_record = [
            "/NoRew_NoInv/monitoring_node/localization",
            "/NoRew_NoInv/monitoring_node/real_queue",
            "/NoRew_NoInv/monitoring_node/temperature",
            "/NoRew_NoInv/monitoring_node/low_temperature",
            "/NoRew_NoInv/perturbation_node/action_performance"]

    def generateOutput(self, normal_time_init, perturbation_time_init, normal_bag_name, pertubation_bag_name, base_init_time_on_first_value=False):
        # ====== Get data from unperturbed experiment ======
        bag = rosbag.Bag(common_experiment_utils.BAG_DIRECTORY_PATH + normal_bag_name + ".bag")

        # Get init time if not set
        if base_init_time_on_first_value:
            for topic, msg, t in bag.read_messages(topics=["/NoRew_NoInv/perturbation_node/action_performance"]):
                # Use the first message time as the init time
                normal_time_init = t.to_sec()
                break

        # Get action performances
        normal_action_performances = common_experiment_utils.ActionPerformanceSeries()
        normal_action_performances.populateWithBag(bag, "/NoRew_NoInv/", normal_time_init)

        # Get performance metrics 
        normal_all_metric_performance_structs = common_experiment_utils.AllMetricPerformanceStruct()
        normal_all_metric_performance_structs.populateWithBag(bag, "/NoRew_NoInv/", normal_time_init)
        
        # ====== Get data from perturbed experiment ======
        bag = rosbag.Bag(common_experiment_utils.BAG_DIRECTORY_PATH + pertubation_bag_name + ".bag")
        
        # Get init time if not set
        if base_init_time_on_first_value:
            for topic, msg, t in bag.read_messages(topics=["/NoRew_NoInv/perturbation_node/action_performance"]):
                # Use the first message time as the init time
                perturbation_time_init = t.to_sec()
                break

        # Get action performances
        perturbation_action_performances = common_experiment_utils.ActionPerformanceSeries()
        perturbation_action_performances.populateWithBag(bag, "/NoRew_NoInv/", perturbation_time_init)

        # Get performance metrics 
        perturbation_all_metric_performance_structs = common_experiment_utils.AllMetricPerformanceStruct()
        perturbation_all_metric_performance_structs.populateWithBag(bag, "/NoRew_NoInv/", perturbation_time_init)
        
        normal_task_queue_departure_arrival_mean_diff = common_experiment_utils.Series()
        normal_task_queue_departure_arrival_mean_diff.variable_name = "Departure and arrival diff NoRew_NoInv"
        normal_task_queue_departure_arrival_mean_diff.values = [departure - arrival for arrival, departure in zip(normal_all_metric_performance_structs.real_queue.queue_server_arrival_mean.values, normal_all_metric_performance_structs.real_queue.target_value.values)]
        
        perturbation_task_queue_departure_arrival_mean_diff = common_experiment_utils.Series()
        perturbation_task_queue_departure_arrival_mean_diff.variable_name = "Departure and arrival diff NoRew_NoInv with perturbation"
        perturbation_task_queue_departure_arrival_mean_diff.values = [departure - arrival for arrival, departure in zip(perturbation_all_metric_performance_structs.real_queue.queue_server_arrival_mean.values, perturbation_all_metric_performance_structs.real_queue.target_value.values)]

        # ====== Create output CSV ======
        normal_data_separator = common_experiment_utils.Series()
        normal_data_separator.variable_name = "Normal NoRew_NoInv"
        perturbation_data_separator = common_experiment_utils.Series()
        perturbation_data_separator.variable_name = "NoRew_NoInv with perturbation"

        csv_filename = common_experiment_utils.RESULT_DIRECTORY_PATH + "normal_experiment2_" + self.string_time_now + ".csv"
        series_to_record = [normal_data_separator,
                            normal_all_metric_performance_structs.localization.time_stamps,
                            normal_all_metric_performance_structs.localization.real_continuous_average_diff_with_target,
                            normal_all_metric_performance_structs.localization.absolute_real_continuous_average_diff_with_server_mean,
                            normal_all_metric_performance_structs.temperature.time_stamps,
                            normal_all_metric_performance_structs.temperature.real_continuous_average_diff_with_target,
                            normal_all_metric_performance_structs.temperature.absolute_real_continuous_average_diff_with_server_mean,
                            normal_all_metric_performance_structs.low_temperature.real_continuous_average_diff_with_target,
                            normal_all_metric_performance_structs.real_queue.time_stamps,
                            normal_all_metric_performance_structs.real_queue.queue_server_arrival_mean,
                            normal_all_metric_performance_structs.real_queue.target_value,
                            normal_task_queue_departure_arrival_mean_diff,
                            normal_action_performances.time_stamps,
                            common_experiment_utils.actionSeriesToStringSeries(normal_action_performances.applied_action),
                            common_experiment_utils.actionSeriesToStringSeries(normal_action_performances.selected_action),
                            normal_action_performances.action_index_difference,
                            perturbation_data_separator,
                            perturbation_all_metric_performance_structs.localization.time_stamps,
                            perturbation_all_metric_performance_structs.localization.real_continuous_average_diff_with_target,
                            perturbation_all_metric_performance_structs.localization.absolute_real_continuous_average_diff_with_server_mean,
                            perturbation_all_metric_performance_structs.temperature.time_stamps,
                            perturbation_all_metric_performance_structs.temperature.real_continuous_average_diff_with_target,
                            perturbation_all_metric_performance_structs.temperature.absolute_real_continuous_average_diff_with_server_mean,
                            perturbation_all_metric_performance_structs.low_temperature.real_continuous_average_diff_with_target,
                            perturbation_all_metric_performance_structs.real_queue.time_stamps,
                            perturbation_all_metric_performance_structs.real_queue.queue_server_arrival_mean,
                            perturbation_all_metric_performance_structs.real_queue.target_value,
                            perturbation_task_queue_departure_arrival_mean_diff,
                            perturbation_action_performances.time_stamps,
                            common_experiment_utils.actionSeriesToStringSeries(perturbation_action_performances.applied_action),
                            common_experiment_utils.actionSeriesToStringSeries(perturbation_action_performances.selected_action),
                            perturbation_action_performances.action_index_difference]
        
        common_experiment_utils.createCSV(series_to_record, csv_filename)

        # ==== Create plots ====
        fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
        fig1.suptitle("Estimation error of the queue server with the real metric mean for the localization metric of a min drift plus penalty with and without action uncertainties")
        
        ax1.set_title("Error between the real metric mean and the estimated metric mean from the queue server metric over time")
        ax1.plot(normal_all_metric_performance_structs.localization.time_stamps.values,
                 normal_all_metric_performance_structs.localization.absolute_real_continuous_average_diff_with_server_mean.values,
                 label="Without perturbation")
        ax1.plot(perturbation_all_metric_performance_structs.localization.time_stamps.values,
                 perturbation_all_metric_performance_structs.localization.absolute_real_continuous_average_diff_with_server_mean.values,
                 label="With perturbations")
        ax1.set_ylabel("Estimation error (m)")
        ax1.grid(False)
        ax1.legend()
        
        ax2.set_title("Error between the real metric mean and the mean target for the localization")
        ax2.plot(normal_all_metric_performance_structs.localization.time_stamps.values,
                 normal_all_metric_performance_structs.localization.real_continuous_average_diff_with_target.values,
                 label="Without perturbation")
        ax2.plot(perturbation_all_metric_performance_structs.localization.time_stamps.values,
                 perturbation_all_metric_performance_structs.localization.real_continuous_average_diff_with_target.values,
                 label="With perturbations")
        ax2.set_ylabel("Error (m)")
        ax2.grid(False)
        ax2.legend()
        ax2.set_xlabel("Time (s)")

        fig2, (ax3, ax4) = plt.subplots(2, 1, sharex=True)
        fig2.suptitle("Estimation error of the queue server with the real metric mean for the temperature metric of a min drift plus penalty with and without action uncertainties")

        ax3.set_title("Absolute error between the real metric mean and the estimated metric mean from the queue server metric over time")
        ax3.plot(normal_all_metric_performance_structs.temperature.time_stamps.values,
                    normal_all_metric_performance_structs.temperature.absolute_real_continuous_average_diff_with_server_mean.values,
                    label="Without perturbation")
        ax3.plot(perturbation_all_metric_performance_structs.temperature.time_stamps.values,
                    perturbation_all_metric_performance_structs.temperature.absolute_real_continuous_average_diff_with_server_mean.values,
                    label="With perturbations")
        ax3.set_ylabel("Estimation error (C)")
        ax3.grid(False)
        ax3.legend()

        ax4.set_title("Absolute error between the real metric mean and the mean target for the temperature")
        ax4.plot(normal_all_metric_performance_structs.temperature.time_stamps.values,
                    normal_all_metric_performance_structs.temperature.real_continuous_average_diff_with_target.values,
                    label="Without perturbation")
        ax4.plot(perturbation_all_metric_performance_structs.temperature.time_stamps.values,
                    perturbation_all_metric_performance_structs.temperature.real_continuous_average_diff_with_target.values,
                    label="With perturbations")
        ax4.set_ylabel("Error (C)")
        ax4.grid(False)
        ax4.legend()
        ax4.set_xlabel("Time (s)")

        fig3, (ax5, ax6) = plt.subplots(2, 1, sharex=True)
        fig3.suptitle("Estimation error of the queue server with the real metric mean for the low temperature metric of a min drift plus penalty with and without action uncertainties")

        ax5.set_title("Absolute error between the real metric mean and the estimated metric mean from the queue server metric over time")
        ax5.plot(normal_all_metric_performance_structs.low_temperature.time_stamps.values,
                    normal_all_metric_performance_structs.low_temperature.real_continuous_average_diff_with_server_mean.values,
                    label="Without perturbation")
        ax5.plot(perturbation_all_metric_performance_structs.low_temperature.time_stamps.values,
                    perturbation_all_metric_performance_structs.low_temperature.real_continuous_average_diff_with_server_mean.values,
                    label="With perturbations")
        ax5.set_ylabel("Estimation error (C)")
        ax5.grid(False)
        ax5.legend()

        ax6.set_title("Absolute error between the real metric mean and the mean target for the low temperature")
        ax6.plot(normal_all_metric_performance_structs.low_temperature.time_stamps.values,
                    normal_all_metric_performance_structs.low_temperature.real_continuous_average_diff_with_target.values,
                    label="Without perturbation")
        ax6.plot(perturbation_all_metric_performance_structs.low_temperature.time_stamps.values,
                    perturbation_all_metric_performance_structs.low_temperature.real_continuous_average_diff_with_target.values,
                    label="With perturbations")
        ax6.set_ylabel("Error (C)")
        ax6.grid(False)
        ax6.legend()
        ax6.set_xlabel("Time (s)")

        fig4, ax7 = plt.subplots(1, 1, sharex=True)
        fig4.suptitle("Task queue departure mean difference with its arrivals mean for a min drift plus penalty with and without action uncertainties")
        ax7.set_title("Difference between the queue server arrival mean and the target value over time")
        
        ax7.plot(normal_all_metric_performance_structs.real_queue.time_stamps.values,
                    normal_task_queue_departure_arrival_mean_diff.values,
                    label="Without perturbation")
        ax7.plot(perturbation_all_metric_performance_structs.real_queue.time_stamps.values,
                    perturbation_task_queue_departure_arrival_mean_diff.values,
                    label="With perturbations")
        ax7.set_ylabel("Difference (Task/s)")
        ax7.grid(False)
        ax7.legend()
        ax7.set_xlabel("Time (s)")
        plt.show()

    def getNormalBagName(self):
        return self.normal_bag_name
    
    def getPerturbationBagName(self):
        return self.perturbation_bag_name
    
    def getNormalROSBagArguments(self):
        topic_args = ""
        for topic in self.topics_to_record:
            topic_args += " " + topic
        
        return "-O "+ common_experiment_utils.BAG_DIRECTORY_PATH + self.normal_bag_name + ".bag" + topic_args

    def getPerturbationROSBagArguments(self):
        topic_args = ""
        for topic in self.topics_to_record:
            topic_args += " " + topic
        
        return "-O "+ common_experiment_utils.BAG_DIRECTORY_PATH + self.perturbation_bag_name + ".bag" + topic_args

