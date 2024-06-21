#!/usr/bin/env python

import os
import rospy
import roslaunch
import rosbag
from datetime import datetime
import csv
from ros_queue_msgs.msg import QueueServerStats

import common_experiment_utils
import experiment_instance
import experiment1_definition
import experiment2_definition
import experiment3_definition

import matplotlib.pyplot as plt
import scienceplots

# Using the formating from https://github.com/garrettj403/SciencePlots to follow IEEE requirements
plt.style.use('science')
# plt.style.use(['science', 'ieee'])

if __name__ == "__main__":
    # Start the launch file
    rospy.init_node('experiment_manager', anonymous=False)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Experiment 1

    cli_args = ['experiment_setup:=perfect_model_and_setup']

    exp1_analyser = experiment1_definition.Experiment1Analyser()
    exp1 = experiment_instance.ExperimentInstance(uuid, cli_args, 60, exp1_analyser)
    #exp1.execute(generate_output=True)
    exp1.analyser.generateOutput(0, "experiment1_2024-06-21_13-03-07", base_init_time_on_first_value=True)
    rospy.loginfo("Experiment 1 completed")

    # Experiment 2
    exp2_analyser = experiment2_definition.Experiment2Analyser()
    exp2 = experiment_instance.Experiment2Instance(uuid, 90, exp2_analyser)
    #exp2.execute(generate_output=True)
    exp2.analyser.generateOutput(0,0, "normal_experiment2_2024-06-21_13-04-13", "perturbation_experiment2_2024-06-21_13-04-13",base_init_time_on_first_value=True)
    rospy.loginfo("Experiment 2 completed")

    # Experiment 3
    # Graph experiment
    exp3_analyser = experiment3_definition.Experiment3Analyser()
    exp3 = experiment_instance.ExperimentInstance(uuid, ['experiment_setup:=bad_prediction_model'], 90, exp3_analyser)
    #exp3.execute(generate_output=True)
    exp3.analyser.generateOutput(0, "experiment3_2024-06-21_13-07-26", base_init_time_on_first_value=True)
    rospy.loginfo("Experiment 3 completed")

    plt.show()
    
     
    


