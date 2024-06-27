#!/usr/bin/env python
import rospy
import roslaunch

import common_experiment_utils
import experiment3_definition

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
        
class Experiment3Instance:
    def __init__(self, uuid, duration_sec, macro_analyser):
        self.duration_sec = duration_sec
        self.macro_analyser = macro_analyser

        self.setup_analysers = {}
        self.parents = []
        
        for setup_name in common_experiment_utils.experimental_setup_list:
            self.setup_analysers[setup_name] = (experiment3_definition.SubExperiment3Analyser(setup_name, self.macro_analyser.getStringTimeNow() ))
            this_analyser = self.setup_analysers[setup_name]

            cli_args = [common_experiment_utils.LAUNCH_DIRECTORY_PATH + "experiment_launcher.launch"] + ['experiment_setup:=' + setup_name, "rosbag_args:=" + this_analyser.getROSBagArguments()]
        
            if setup_name == "all_faults_without_penalty":
                cli_args.append('v_parameter:=0.0')

            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0],roslaunch_args)]

            self.parents.append(roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file))

    def execute(self, generate_output=True):
        for parent in self.parents:
            time_init = rospy.Time.now()
            parent.start()
            rospy.sleep(self.duration_sec)
            parent.shutdown()
        
        # Analyse results
        if generate_output:
            self.generateAllSubExperimentOutputs(self.macro_analyser.getStringTimeNow())
    
    def generateAllSubExperimentOutputs(self, time_prefix):
        for setup_name in common_experiment_utils.experimental_setup_list:
            this_analyser = self.setup_analysers[setup_name]
            bag_name = "experiment3_" + setup_name + "_" + time_prefix
            if setup_name == "bad_prediction_model":
                this_analyser.generateOutput(0, bag_name, base_init_time_on_first_value=True)
            else:
                this_analyser.generateOutput(0, bag_name, generate_plots=False,  base_init_time_on_first_value=True)
                
        
