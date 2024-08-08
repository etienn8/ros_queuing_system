#!/usr/bin/env python
import rospy

import os
import rosbag
from datetime import datetime
import csv
import matplotlib.pyplot as plt
from typing import Dict

import common_experiment_utils

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

                if metric_name == "penalty":
                    controller_end_struct.metric[metric_name].mean_value.values = [controller_performance_metric_dict[metric_name].real_average_value.values[-1]]
                    controller_end_struct.metric[metric_name].estimation_error.values = [controller_performance_metric_dict[metric_name].real_continuous_average_diff_with_server_mean.values[-1]]
                else:
                    controller_end_struct.metric[metric_name].target_error.values = [controller_performance_metric_dict[metric_name].target_diff_with_real_continuous_average.values[-1]]
                    if controller_type == "Rew_NoInv" or controller_type == "Rew_Inv":
                        controller_end_struct.metric[metric_name].estimation_error.values = [controller_performance_metric_dict[metric_name].absolute_real_continuous_average_diff_with_server_time_average.values[-1]]
                    else:
                        controller_end_struct.metric[metric_name].estimation_error.values = [controller_performance_metric_dict[metric_name].absolute_real_continuous_average_diff_with_server_mean.values[-1]]

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
            if controller_type == "Rew_NoInv" or controller_type == "Rew_Inv":
                series_to_record += [controller_separator,
                                    controller_performance_metrics[controller_type].localization.time_stamps,
                                    controller_performance_metrics[controller_type].localization.real_continous_average_value,
                                    controller_performance_metrics[controller_type].localization.target_value,
                                    controller_performance_metrics[controller_type].localization.absolute_real_continuous_average_diff_with_server_time_average,
                                    controller_performance_metrics[controller_type].localization.target_diff_with_real_continuous_average,
                                    controller_performance_metrics[controller_type].temperature.time_stamps,
                                    controller_performance_metrics[controller_type].temperature.real_continous_average_value,
                                    controller_performance_metrics[controller_type].temperature.target_value,
                                    controller_performance_metrics[controller_type].temperature.absolute_real_continuous_average_diff_with_server_time_average,
                                    controller_performance_metrics[controller_type].temperature.target_diff_with_real_continuous_average,
                                    controller_performance_metrics[controller_type].low_temperature.target_value,
                                    controller_performance_metrics[controller_type].low_temperature.target_diff_with_real_continuous_average,
                                    controller_performance_metrics[controller_type].real_queue.time_stamps,
                                    controller_performance_metrics[controller_type].real_queue.queue_server_arrival_mean,
                                    controller_performance_metrics[controller_type].real_queue.target_value,
                                    controller_performance_metrics[controller_type].real_queue.absolute_real_continuous_average_diff_with_server_mean,
                                    controller_performance_metrics[controller_type].penalty.time_stamps,
                                    controller_performance_metrics[controller_type].penalty.real_time_average_value]
            else:
                [controller_separator,
                controller_performance_metrics[controller_type].localization.time_stamps,
                controller_performance_metrics[controller_type].localization.real_continous_average_value,
                controller_performance_metrics[controller_type].localization.target_value,
                controller_performance_metrics[controller_type].localization.absolute_real_continuous_average_diff_with_server_mean,
                controller_performance_metrics[controller_type].localization.target_diff_with_real_continuous_average,
                controller_performance_metrics[controller_type].temperature.time_stamps,
                controller_performance_metrics[controller_type].temperature.real_continous_average_value,
                controller_performance_metrics[controller_type].temperature.target_value,
                controller_performance_metrics[controller_type].temperature.absolute_real_continuous_average_diff_with_server_mean,
                controller_performance_metrics[controller_type].temperature.target_diff_with_real_continuous_average,
                controller_performance_metrics[controller_type].low_temperature.target_value,
                controller_performance_metrics[controller_type].low_temperature.target_diff_with_real_continuous_average,
                controller_performance_metrics[controller_type].real_queue.time_stamps,
                controller_performance_metrics[controller_type].real_queue.queue_server_arrival_mean,
                controller_performance_metrics[controller_type].real_queue.target_value,
                controller_performance_metrics[controller_type].real_queue.absolute_real_continuous_average_diff_with_server_mean,
                controller_performance_metrics[controller_type].penalty.time_stamps,
                controller_performance_metrics[controller_type].penalty.real_time_average_value]
                
        
        common_experiment_utils.createCSV(series_to_record, csv_filename)

        if generate_plots:
            # Create output figures director        
            figure_directory = common_experiment_utils.RESULT_DIRECTORY_PATH + self.bag_name
            if os.path.exists(figure_directory) == False:
                os.mkdir(figure_directory)


            # ==== Create plots ====
            fig1, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
            #ax1.set_title("Estimation error between the real metric mean and the queue server mean for the localization metric for all controllers over time with " + self.setup_name)
            for controller_type in common_experiment_utils.controller_type_list:
                metric_performance_struct = controller_performance_metrics[controller_type].localization
                if controller_type == "Rew_NoInv" or controller_type == "Rew_Inv":
                    ax1.plot(metric_performance_struct.time_stamps.values, 
                            metric_performance_struct.absolute_real_continuous_average_diff_with_server_time_average.values, 
                            label=common_experiment_utils.controllerTypePaperConversion(controller_type))
                else:
                    ax1.plot(metric_performance_struct.time_stamps.values, 
                            metric_performance_struct.absolute_real_continuous_average_diff_with_server_mean.values, 
                            label=common_experiment_utils.controllerTypePaperConversion(controller_type))
        
            ax1.set_ylabel("Estimation error (cm)")
            ax1.grid(True)
            ax1.legend()

            #ax2.set_title("Error between the real metric mean and the target value for the localization metric for all controllers over time with " + self.setup_name)
            for controller_type in common_experiment_utils.controller_type_list:
                metric_performance_struct = controller_performance_metrics[controller_type].localization
                ax2.plot(metric_performance_struct.time_stamps.values, 
                        metric_performance_struct.target_diff_with_real_continuous_average.values, 
                        label=common_experiment_utils.controllerTypePaperConversion(controller_type))
            
            ax2.set_ylabel("Command error (cm)")
            ax2.grid(True)
            #ax2.legend()
            ax2.set_xlabel("Time (s)")

            fig1.savefig(figure_directory+"/localization_error.pdf", format="pdf", bbox_inches='tight')


            fig2, (ax3, ax4) = plt.subplots(2, 1, sharex=True)
            #ax3.set_title("Estimation error between the real metric mean and the queue server mean for the temperature metric for all controllers over time with " + self.setup_name)
            for controller_type in common_experiment_utils.controller_type_list:
                metric_performance_struct = controller_performance_metrics[controller_type].temperature
                if controller_type == "Rew_NoInv" or controller_type == "Rew_Inv":
                    ax3.plot(metric_performance_struct.time_stamps.values, 
                            metric_performance_struct.absolute_real_continuous_average_diff_with_server_time_average.values, 
                            label=common_experiment_utils.controllerTypePaperConversion(controller_type))
                else:
                    ax3.plot(metric_performance_struct.time_stamps.values, 
                            metric_performance_struct.absolute_real_continuous_average_diff_with_server_mean.values, 
                            label=common_experiment_utils.controllerTypePaperConversion(controller_type))
        
            ax3.set_ylabel("Estimation error (°C)")
            ax3.grid(True)
            ax3.legend()

            #ax4.set_title("Error between the real metric mean and the target value for the temperature metric for all controllers over time with " + self.setup_name)
            for controller_type in common_experiment_utils.controller_type_list:
                metric_performance_struct = controller_performance_metrics[controller_type].temperature
                ax4.plot(metric_performance_struct.time_stamps.values, 
                        metric_performance_struct.target_diff_with_real_continuous_average.values, 
                        label=common_experiment_utils.controllerTypePaperConversion(controller_type))
            
            ax4.set_ylabel("Command error (°C)")
            ax4.grid(True)
            #ax4.legend()
            ax4.set_xlabel("Time (s)")
            fig2.savefig(figure_directory+"/temperature_error.pdf", format="pdf", bbox_inches='tight')

    def prefixToBagName(self, prefix):
        self.bag_name = prefix + self.bag_name

    def getBagName(self):
        return self.bag_name
    
    def getROSBagArguments(self):
        topic_args = ""
        for topic in self.topics_to_record:
            topic_args += " " + topic
        
        return "-O "+ common_experiment_utils.BAG_DIRECTORY_PATH + self.bag_name + ".bag" + topic_args

class Experiment3Analyser:
    def __init__(self):
        time_now = datetime.now()
        self.string_time_now = time_now.strftime("%Y-%m-%d_%H-%M-%S")
        self.multi_setup_end_values = common_experiment_utils.MultiSetupEndMetricStruct()

    def getStringTimeNow(self):
        return self.string_time_now
    
    def generateMultiSetupOutputs(self, analyser_dict: Dict[str, SubExperiment3Analyser]):
        for setup_name, analyser in analyser_dict.items():
            self.multi_setup_end_values.multi_controller_end_struct[setup_name] = analyser.multi_controller_end_struct
        
        # ======= Create output CSV =======

        # Create variable names
        nb_lines = 6 * 4 # 8 fields per controller and there are 4 controllers 
        controller_name_spacers = common_experiment_utils.Series()
        controller_name_spacers.variable_name = "Controller type"
        metric_name_spacers = common_experiment_utils.Series()
        metric_name_spacers.variable_name = "Metric type"
        error_type_spaces = common_experiment_utils.Series()
        error_type_spaces.variable_name = ""

        setup_series = {}
        for setup_name in common_experiment_utils.experimental_setup_list:
            setup_series[setup_name] = common_experiment_utils.Series()
            setup_series[setup_name].variable_name = setup_name

        
        for index in range(nb_lines):
            if index == 0:
                controller_name_spacers.values.append("NoRew_NoInv")
            elif index == 6:
                controller_name_spacers.values.append("NoRew_Inv")
            elif index == 12:
                controller_name_spacers.values.append("Rew_NoInv")
            elif index == 18:
                controller_name_spacers.values.append("Rew_Inv")
            else:
                controller_name_spacers.values.append("")

            metric_index = index % 6
            if metric_index == 0:
                metric_name_spacers.values.append("Localization")
                error_type_spaces.values.append("Estimation error (cm)")
            elif metric_index == 1:
                metric_name_spacers.values.append("")
                error_type_spaces.values.append("Target error (cm)")
            elif metric_index == 2:
                metric_name_spacers.values.append("Temperature")
                error_type_spaces.values.append("Estimation error (°C)")
            elif metric_index == 3:
                metric_name_spacers.values.append("")
                error_type_spaces.values.append("Target error (°C)")
#            elif metric_index == 4:
#                metric_name_spacers.values.append("Low Temperature")
#                error_type_spaces.values.append("Target error (°C)")
            elif metric_index == 4:
                metric_name_spacers.values.append("Real Queue")
                error_type_spaces.values.append("Departure-arrival diff (Task/s)")
            elif metric_index == 5:
                metric_name_spacers.values.append("Penalty")
                error_type_spaces.values.append("Penalty mean (J)")
                #error_type_spaces.values.append("Penalty estimation error (J)")
            else:
                metric_name_spacers.values.append("")


        series_to_record = [controller_name_spacers, 
                            metric_name_spacers,
                            error_type_spaces]

        for setup_name in common_experiment_utils.experimental_setup_list:
            for controller_type in common_experiment_utils.controller_type_list:
                controller_end_metrics = self.multi_setup_end_values.multi_controller_end_struct[setup_name].controller_end_metrics[controller_type]
                
                setup_series[setup_name].values.append(round(controller_end_metrics.metric["localization"].estimation_error.values[0],3))
                setup_series[setup_name].values.append(round(controller_end_metrics.metric["localization"].target_error.values[0],3))
                setup_series[setup_name].values.append(round(controller_end_metrics.metric["temperature"].estimation_error.values[0],3))
                setup_series[setup_name].values.append(round(controller_end_metrics.metric["temperature"].target_error.values[0],3))
                #setup_series[setup_name].values.append(round(controller_end_metrics.metric["low_temperature"].target_error.values[0],3))
                setup_series[setup_name].values.append(round(controller_end_metrics.metric["real_queue"].target_error.values[0],3))
                #setup_series[setup_name].values.append(round(controller_end_metrics.metric["penalty"].estimation_error.values[0],3))
                setup_series[setup_name].values.append(round(controller_end_metrics.metric["penalty"].mean_value.values[0],3))
            series_to_record.append(setup_series[setup_name])

  
        csv_filename = common_experiment_utils.RESULT_DIRECTORY_PATH + "exp3_end_values_" + self.getStringTimeNow()  + ".csv"
        common_experiment_utils.createCSV(series_to_record, csv_filename)
        