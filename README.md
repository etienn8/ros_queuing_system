# ROS Queuing System

## Overview
This repos contains all the necessary components to build a stabilizing queuing system based on stochastic network optimization in ROS[1]. 

This implementation stores ROS messages in queues and uses virtual queues to represent any other scalar metrics that needs to be controlled (ex: temperature, connectivity, battery state of charge,...). When combined with a queue controller, an optimization framework can be formulated where an action from a set of action will be taken to optimize a given metric and to stabilized all the queues (if feasible). 

The stabilization of the queue means that the expected time average size of the queue will be fixed. In other words, this framework will greedily find an action at each time step, so in general (related to the time average), the given metrics constraint will be respected and the desired delay in the transmission of the real queues will be respected.

This gives a general and soft multi-objective decision making framework in robotics where some metrics might be sacrificed for another metric at each time step but the overall given average time constraints will be respected.

All this implementation is based on the work of Neely [1]. The core concepts all also shown in the [queue_controller](https://github.com/etienn8/ros_queuing_system/tree/main/queue_controller) package.

## System components
![](queue_controller/.assets/ros_queuing_system_architecture.png)

## How to get started?
Here's a procedure on how to get started with the configurations and the launching of the queuing system.
### Building from Source
#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics).
- [queue_controller](https://github.com/etienn8/ros_queuing_system/tree/main/queue_controller) (Server of queues, included already in the ros_queuing_system repo).
- [queue_server](https://github.com/etienn8/ros_queuing_system/tree/main/queue_server) (Server of queues, included already in the ros_queuing_system repo).
- [ros_queue](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue) (Libraries of queues, included already in the ros_queuing_system repo).
- [ros_queue_msgs](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue_msgs) (ROS messages and services used as interface for the ros_queuing system, included already in the ros_queuing_system repo).
- [ros_queue_msgs](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue_msgs) (Implementation examples and dummy stochastic services to test the system, included already in the ros_queuing_system repo).
- [rosparam_utils](https://github.com/etienn8/rosparam_utils) (Tools to fetch rosparams more easily. Not directly dependent but some dependent packages need it).
- [ros_boosted_utilities](https://github.com/etienn8/ros_boosted_utilities) (Contains persistent ROS service clients to make the interfaces between the system faster.)
##### Optional
- [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/#)(Tools to help build all the packages in parallel and to test the system)
	- Follow their [installation](https://catkin-tools.readthedocs.io/en/latest/installing.html) and [initialization](https://catkin-tools.readthedocs.io/en/latest/quick_start.html) guide before building this package.


#### Building

To build from source, clone the [rosparam_utils](https://github.com/etienn8/rosparam_utils), clone the [ros_boosted_utilities](https://github.com/etienn8/ros_boosted_utilities) repo, clone the latest version from this repository into your catkin workspace and compile all the packages using

	cd catkin_workspace/src
	git clone https://github.com/etienn8/rosparam_utils.git
	git clone https://github.com/etienn8/ros_boosted_utilities.git
	git clone https://github.com/etienn8/ros_queuing_system.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

If you're using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html), you could use `catkin build` command instead of `catkin_make`.

### Starting a queue server
Follow the [queue_server](https://github.com/etienn8/ros_queuing_system/tree/main/queue_server)'s readme to configure the queues and the sever, and then start the server.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/etienn8/ros_queuing_system/issues).

# References
[1] M. J. Neely, *Stochastic Network Optimization with Application to Communication and Queueing Systems*, 1st ed. Cham: Springer Cham, 2022.