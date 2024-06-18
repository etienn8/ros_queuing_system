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

    cli_args = [common_experiment_utils.LAUNCH_DIRECTORY_PATH + "experiment_launcher.launch", 'experiment_setup:=perfect_model_and_setup']

    exp1_analyser = experiment1_definition.Experiment1Analyser()
    exp1 = experiment_instance.ExperimentInstance(uuid, cli_args, 60, exp1_analyser)
    exp1.execute(generate_output=False)
    #exp1.analyser.generateOutput(0, "experiment1_2024-06-10_14-55-02", base_init_time_on_first_value=True)
    exp1.analyser.generateOutput(exp1.time_init.to_sec(), exp1.analyser.getBagName())
    rospy.loginfo("Experiment 1 completed")
    


