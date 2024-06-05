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

if __name__ == "__main__":
    # Start the launch file
    rospy.init_node('experiment_manager', anonymous=False)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Experiment 1

    cli_args = [common_experiment_utils.LAUNCH_DIRECTORY_PATH + "experiment_launcher.launch", 'experiment_setup:=perfect_model_and_setup']

    exp1_analyser = experiment1_definition.Experiment1Analyser()
    exp1 = experiment_instance.ExperimentInstance(uuid, cli_args, 10, exp1_analyser)
    exp1.execute()
    rospy.loginfo("Experiment 1 completed")

