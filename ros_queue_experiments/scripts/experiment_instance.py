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

    def execute(self, generate_output=True):
        self.time_init = rospy.Time.now()
        self.parent.start()
        rospy.sleep(self.duration_sec)
        self.parent.shutdown()
        # Analyse results
        if generate_output:
            self.analyser.generateOutput(self.time_init.to_sec(), self.analyser.getBagName())



class Experiment2Instance:
    def __init__(self, uuid, duration_sec, analyser):

        self.duration_sec = duration_sec
        
        self.analyser = analyser
        
        normal_cli_args = [common_experiment_utils.LAUNCH_DIRECTORY_PATH + "experiment_launcher.launch"] + ['experiment_setup:=perfect_model_and_setup', "rosbag_args:=" + self.analyser.getNormalROSBagArguments()]
        normal_roslaunch_args = normal_cli_args[1:]
        normal_roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(normal_cli_args)[0],
                                  normal_roslaunch_args)]

        self.normal_parent = roslaunch.parent.ROSLaunchParent(uuid, normal_roslaunch_file)

        perturbation_cli_args = [common_experiment_utils.LAUNCH_DIRECTORY_PATH + "experiment_launcher.launch"] + ['experiment_setup:=perturbation_on_action', "rosbag_args:=" + self.analyser.getPerturbationROSBagArguments()]
        perturbation_roslaunch_args = perturbation_cli_args[1:]
        perturbation_roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(perturbation_cli_args)[0],
                                        perturbation_roslaunch_args)]

        self.perturbation_parent = roslaunch.parent.ROSLaunchParent(uuid, perturbation_roslaunch_file)

    def execute(self, generate_output=True):
        self.normal_time_init = rospy.Time.now()
        self.normal_parent.start()
        rospy.sleep(self.duration_sec)
        self.normal_parent.shutdown()

        self.perturbation_time_init = rospy.Time.now()
        self.perturbation_parent.start()
        rospy.sleep(self.duration_sec)
        self.perturbation_parent.shutdown()
        
        # Analyse results
        if generate_output:
            self.analyser.generateOutput(self.normal_time_init.to_sec(), 
                                         self.perturbation_time_init.to_sec(), 
                                         self.analyser.getNormalBagName(), 
                                         self.analyser.getPerturbationBagName(), 
                                         base_init_time_on_first_value=False)

            

