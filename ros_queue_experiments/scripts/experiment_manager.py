#!/usr/bin/env python

import os
import rospy
import roslaunch

# Get the path of the launch file
launch_file = "experiment_launcher.launch"
script_path_list = os.path.normpath(os.path.abspath(__file__)).split(os.sep)
launch_directory_path_list = script_path_list[:-2]
launch_directory_path_list.append("launch")

launch_directory_path = os.sep.join(launch_directory_path_list)

full_launch_path = launch_directory_path + "/" + launch_file

# Start the launch file
rospy.init_node('experiment_manager', anonymous=False)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)


# Experiment 1 
exp1_duration_sec = 120
cli_args = [full_launch_path, 'experiment_setup:=perfect_model_and_setup']
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

parent.start()

rospy.sleep(5)

parent.shutdown()

# Is parent still shuting down?
parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
parent.start()

rospy.sleep(exp1_duration_sec)

parent.shutdown()

