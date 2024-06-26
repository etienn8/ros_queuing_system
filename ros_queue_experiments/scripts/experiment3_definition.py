#!/usr/bin/env python
import rospy

import rosbag
from datetime import datetime
import csv
import matplotlib.pyplot as plt

import common_experiment_utils

class Experiment3Analyser:
    def __init__(self):
        time_now = datetime.now()
        self.string_time_now = time_now.strftime("%Y-%m-%d_%H-%M-%S")

    def getStringTimeNow(self):
        return self.string_time_now
        

class SubExperiment3Analyser:
    def __init__(self, setup_name, string_time_now):
        self.bag_name = "experiment3_" + setup_name + "_" + string_time_now
        self.setup_name = setup_name
        self.sub_topics_to_record = [
            "queue_controller/control_loop_started",
            "monitoring_node/localization",
            "monitoring_node/real_queue",
            "monitoring_node/temperature",
            "monitoring_node/low_temperature",
            "monitoring_node/penalty"]

        self.topics_to_record = []
        for controller_type in common_experiment_utils.controller_type_list:
            for sub_topic in self.sub_topics_to_record:
                self.topics_to_record.append("/" + controller_type + "/" + sub_topic)

    def generateOutput(self, time_init, bag_name, generate_plots = True, base_init_time_on_first_value=False):
        bag = rosbag.Bag(common_experiment_utils.BAG_DIRECTORY_PATH + bag_name + ".bag")

        for topic, msg, t in bag.read_messages(topics=["/NoRew_NoInv/queue_controller/control_loop_started"]):
            if base_init_time_on_first_value:
                time_init = t.to_sec()
                init_time_set = True
                break

        # Get performance metrics 
        controller_performance_metrics = {"NoRew_NoInv": common_experiment_utils.AllMetricPerformanceStruct(),
                                          "NoRew_Inv": common_experiment_utils.AllMetricPerformanceStruct(),
                                          "Rew_NoInv": common_experiment_utils.AllMetricPerformanceStruct(),
                                          "Rew_Inv": common_experiment_utils.AllMetricPerformanceStruct()}
        
        for controller_type in common_experiment_utils.controller_type_list:
            controller_performance_metrics[controller_type].populateWithBag(bag, "/" + controller_type + "/", time_init)

        # Get end values of the metrics
        self.multi_controller_end_struct = common_experiment_utils.MultiControllerEndMetricStruct()
        for controller_type in common_experiment_utils.controller_type_list:
            controller_performance_metric_dict = {"localization": controller_performance_metrics[controller_type].localization,
                                                  "temperature": controller_performance_metrics[controller_type].temperature,
                                                  "low_temperature": controller_performance_metrics[controller_type].low_temperature,
                                                  "real_queue": controller_performance_metrics[controller_type].real_queue,
                                                  "penalty": controller_performance_metrics[controller_type].penalty}
             
            controller_end_struct = self.multi_controller_end_struct.controller_end_metrics[controller_type]

            for metric_name in common_experiment_utils.metric_type_list:
                controller_end_struct.metric[metric_name].estimation_error.values = [controller_performance_metric_dict[metric_name].absolute_real_continuous_average_diff_with_server_mean.values[-1]]
                controller_end_struct.metric[metric_name].target_error.values = [controller_performance_metric_dict[metric_name].real_continuous_average_diff_with_target.values[-1]]

        # Create output CSV
        separator_second_graph = common_experiment_utils.Series()
        separator_second_graph.variable_name = "controller_sacrifices_one_queue"
        separator_end_values_table = common_experiment_utils.Series()
        separator_end_values_table.variable_name = "end_values_table"

        csv_filename = common_experiment_utils.RESULT_DIRECTORY_PATH + self.bag_name + ".csv"
        series_to_record = []
        for controller_type in common_experiment_utils.controller_type_list:
            controller_separator = common_experiment_utils.Series()
            controller_separator.variable_name = controller_type
            series_to_record += [controller_separator,
                                 controller_performance_metrics[controller_type].localization.time_stamps,
                                 controller_performance_metrics[controller_type].localization.real_continous_average_value,
                                 controller_performance_metrics[controller_type].localization.target_value,
                                 controller_performance_metrics[controller_type].localization.absolute_real_continuous_average_diff_with_server_mean,
                                 controller_performance_metrics[controller_type].localization.real_continuous_average_diff_with_target,
                                 controller_performance_metrics[controller_type].temperature.time_stamps,
                                 controller_performance_metrics[controller_type].temperature.real_continous_average_value,
                                 controller_performance_metrics[controller_type].temperature.target_value,
                                 controller_performance_metrics[controller_type].temperature.absolute_real_continuous_average_diff_with_server_mean,
                                 controller_performance_metrics[controller_type].temperature.real_continuous_average_diff_with_target,
                                 controller_performance_metrics[controller_type].low_temperature.target_value,
                                 controller_performance_metrics[controller_type].low_temperature.real_continuous_average_diff_with_target,
                                 controller_performance_metrics[controller_type].real_queue.time_stamps,
                                 controller_performance_metrics[controller_type].real_queue.queue_server_arrival_mean,
                                 controller_performance_metrics[controller_type].real_queue.target_value,
                                 controller_performance_metrics[controller_type].real_queue.absolute_real_continuous_average_diff_with_server_mean,
                                 controller_performance_metrics[controller_type].penalty.time_stamps,
                                 controller_performance_metrics[controller_type].penalty.real_time_average_value]
        
        common_experiment_utils.createCSV(series_to_record, csv_filename)

        if generate_plots:
            # ==== Create plots ====
            fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
            ax1.set_title("Estimation error between the real metric mean and the queue server mean for the localization metric for all controllers over time with " + self.setup_name)
            for controller_type in common_experiment_utils.controller_type_list:
                metric_performance_struct = controller_performance_metrics[controller_type].localization
                ax1.plot(metric_performance_struct.time_stamps.values, 
                        metric_performance_struct.absolute_real_continuous_average_diff_with_server_mean.values, 
                        label=controller_type)
        
            ax1.set_ylabel("Localization estimation error (m)")
            ax1.grid(True)
            ax1.legend()

            ax2.set_title("Error between the real metric mean and the target value for the localization metric for all controllers over time with model uncertainties" + self.setup_name)
            for controller_type in common_experiment_utils.controller_type_list:
                metric_performance_struct = controller_performance_metrics[controller_type].localization
                ax2.plot(metric_performance_struct.time_stamps.values, 
                        metric_performance_struct.real_continuous_average_diff_with_target.values, 
                        label=controller_type)
            
            ax2.set_ylabel("Localization error (m)")
            ax2.grid(True)
            ax2.legend()
            ax2.set_xlabel("Time (s)")

    def prefixToBagName(self, prefix):
        self.bag_name = prefix + self.bag_name

    def getBagName(self):
        return self.bag_name
    
    def getROSBagArguments(self):
        topic_args = ""
        for topic in self.topics_to_record:
            topic_args += " " + topic
        
        return "-O "+ common_experiment_utils.BAG_DIRECTORY_PATH + self.bag_name + ".bag" + topic_args

