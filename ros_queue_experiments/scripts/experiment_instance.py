#!/usr/bin/env python
import rospy
import roslaunch

import common_experiment_utils

class ExperimentInstance:
    def __init__(self, uuid, experiment_launcher_args, duration_sec, analyser):

        self.analyser = analyser
        cli_args = [common_experiment_utils.LAUNCH_DIRECTORY_PATH + "experiment_launcher.launch"] + experiment_launcher_args + ["rosbag_args:=" + self.analyser.getROSBagArguments()]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        self.duration_sec = duration_sec

        self.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

    def execute(self):
        self.time_init = rospy.Time.now()
        self.parent.start()
        rospy.sleep(self.duration_sec)
        self.parent.shutdown()
        # Analyse results
        self.analyser.generateOutput(self.time_init.to_sec())

