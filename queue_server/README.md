# queue_server

## Overview

This package includes a server that creates and stores queues from the [ros_queue](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue) package. It also offer a service to trigger the update of all its virtual queues and a periodic update to transmit the data from the real queues. Lastly, the queue_server publishes the state (size) of its queues periodically. All the queues are created from an extensible configuration file.

An extra node, *periodic_update_caller_node*, is also included if the update of the queue server needs to be executed at a predefined static rate.

**Keywords:** queuing theory, server

### License
The source code is released under a MIT license.

**Author: Étienne Villemure**
**Affiliation: [MoreLab](https://morelab.ca/) and [IntRoLab](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/Main_Page)**
**Maintainer: Étienne Villemure, etienne.villemure@usherbrooke.ca**

The ros_queuing_system package has been tested under [ROS] Noetic on Ubuntu 20.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation
### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [ros_queue_msgs](https://github.com/etienn8/ros_queuing_system/tree/feat/readme_ros_queue_and_server/ros_queue_msgs)(ROS data structures for the queues interfaces),
- [ros_queue](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue) (Queue data structures used by the queue_server),
- [rosparam_utils](https://github.com/etienn8/rosparam_utils) (Tools to fetch rosparams more easily),
- [std_srvs](http://wiki.ros.org/std_srvs)(Standard ROS services definition),
- std::map (Standard library for C++ dictionnaries),
- std::utility (Standard library that permits move semantics),
- std::memory (Standard library for smart pointers)

	sudo rosdep install --from-paths src
#### Building

To build from source, clone the [rosparam_utils](https://github.com/etienn8/rosparam_utils) repo, clone the latest version from this repository into your catkin workspace and compile all the packages using

	cd catkin_workspace/src
	git clone https://github.com/etienn8/rosparam_utils.git
	git clone https://github.com/etienn8/ros_queuing_system.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

If you're using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html), you could use `catkin build` command instead of `catkin_make`.

## Usage

1. Configure the queues in the `config/queue_server_config_template.yaml`  to create the desired queues. A basic example is provided but it can be modified or another config could be used. Instructions on how to configure the queues is provided in the next section.
3. Start the queue server with: 
	`roslaunch queue_server queue_server_template.launch`

At that point, a queue server will be created. The real queues will grow depending on the configured arrival topic and will decrease periodically if their `transmission_evaluation_service` returns a number higher than the size of if its first message in the queue. The virtual queues will be updated whenever the `<queue_server_name>/trigger_service` will be called and they will changed by their arrival minus the departure evaluation service returned values.

To periodically trigger the update of the virtual queue, do these additional steps:
1. Modify the `update_rate` variable in `launch/periodic_udpate_caller.launch` to the desired rate (calls by second). Make sure that `trigger_service_name` matches the trigger service name of the queue_server.
2. Start the periodic update caller with:
	`roslaunch queue_server periodic_udpate_caller.launch`

To monitor the state of the servers and the queues, you can print them in the terminal with:
`rostopic echo <queue_server_name>/server_state`

## Config files

**queue_config.yaml**: Its a template to copy paste in the **queue_server_config_template.yaml** that lists all the queue parameters
- `Queue_name`(list): the name of this parameter is the name of the param list and constitute in itself, the name of the queue.
- `type_of_queue` (string): It indicates what type of ros_queue this queue is. Depending on this type, some parameter will be omited and some will be needed. There is three possible values: 
	- `real_queue`: It's a real queue that stores ROS messages. Its a [ROSByteConvertedQueue](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue) so the size of the queue will be given in bytes.
	- `inequality_constraint_virtual_queue`: It's a virtual queue with a float size that won't go below 0.
	- `equality_constraint_virtual_queue`: It's a virtual queue with a float size that can go below 0.
- `max_queue_size` (float): A float value for virtual queues and value in bytes for real_queues that set the maximum size of the queue. Data won't be added over this limit and will be discarded.
- Parameters for virtual queues:
	- `arrival_evaluation_service_name` (string): Indicate the service name that the virtual queue will used to know by how much it needs increase its queue when an update is triggered. 
	- `departure_evaluation_service_name` (string): Indicate the service name that the virtual queue will used to know by how much it needs decrease its queue when an update is triggered. 
- Parameters for real queues:
	- `arrival_topic_name` (string): Name of the topic that the real queue will listen to and store message from. It can take any message ROS type as an input.
	- `tranmission_topic_name` (string): Name of the topic on which the queue will transmit its messages. The type of message will match the message type of the first message received on `arrival_topic_name`.
	- `transmission_evaluation_service_name` (string): Name of the service that the real queue will call periodically to know how much data it could transmit in bytes. **WARNING**: if the next element to be sent in the queue as a size bigger that what the service returns, the queue will not transmit.

* **queue_server_config_template.yaml**: This is a template that contains the server parameters and the configuration of its queues.
	* `queue_server_name` (string): Indicates the name of the queue and thus will be used as a prefix for all the ROS interfaces of the server and its queues.
  	* `compute_statistics` (bool, default:false): When true, queues will publish a topic with their time average metrics (queue size, arrivals and departures).
	* `server_spin_rate` (float): The frquency in loops/sec at which the queue sizes will be published and the real queue will be checked for transmission.
	* `queue_list` (list): Its a list of the queue configurations. Any number of queues can be defined in the list. To not forget any queue configurations, copy-paste the content of `queue_config.yaml` in the list to add a new queue and configure it as needed.
## Launch files

* **periodic_udpate_caller.launch:** Starts a node that periodically calls, at a specified rate, the virtual queues update of a queue_server node
     - **`update_rate`** Rate at which the update will be triggered. Default: 2.0

* **queue_server_template.launch:** Starts a queue server based on queue server configuration file.
     - **`server_name`** Name of the server that is used as meta information, the node's name and the prefix of all its interfaces. Default: `queue_server`.
     - **`queue_server_config_path`** Path to the server configuration file. It needs to contain all the needed information like shown in the default config file. Default: "$(find queue_server)/config/queue_server_config_template.yaml"

## Nodes

### periodic_update_caller_node

Periodically calls an empty service. Usually used to trigger the virtual queue update at a specific rate.

#### Service calls

* **`<trigger_service_name>`** ([std_srvs/Empty])

	Calls the empty service and expects no response.

### queue_server_node

Creates [ros_queue](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue) queues based on a given configuration file, stores the queues, publishes periodically the queue states, transmits periodically the real queues based on a given amount of data to transmit given by a service and updates the virtual queues whenever it receives a trigger. In a nutshell, it's responsible to hold queues and it manages the interactions from other nodes with those queues. 

#### Subscribed Topics
* **`virtual_queue_manual_changes`** ([ros_queue_msgs/VirtualQueueChangesList](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/msg/VirtualQueueChangesList.msg))
	Topic that contains a list of virtual queues' name and manual changes that needs to be performed on the specified queues.

For all real queues configured in the config files:
* **`<queue_name>/<arrival_topic_name>`** ([Any type of ROS message])
	Messages from that topic will be stored in its corresponding real queue.

#### Published Topics
* **`server_state`** ((ros_queue_msgs/QueueServerState)[https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/msg/QueueServerState.msg])
	Publishes the states of each queue. It's mainly containing the size of the queues.

* **`server_stats`** ((ros_queue_msgs/QueueServerStats)[https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/msg/QueueServerStats.msg])
	Publishes stats about the mean average metrics of the queues (time average arrivals and departures and mean queue size). Only active if the param `compute_statistics` is set to true.

For all real queues configured in the config files
* **`<queue_name>/<tranmission_topic_name>`** ([Type of first message received on the arrival_topic_name of a real queue])
	Message to transmit from the real queue.

#### Services
* **`trigger_service`** ([std_srvs/Empty])
	Triggers an update of all the virtual queues when called. During updates, the virtual queues will call their arrival and transmission evaluation services to compute the change in the queue size. For example, you can trigger the update from the console with:

		rosservice call /<queue_server_name>/trigger_service

* **`get_server_state`** ([std_queue_msgs/QueueServerStateFetch](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/msg/QueueServerState.msg))
	Returns the current server states. It mainly includes the current queue sizes. For example, you can display the server states from the console with:

		rosservice call /<queue_server_name>/trigger_service

For each queue (virtual and real):
* **`<queue_name>/getQueueInfo`** ([ros_queue_msgs/QueueInfoFetch](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/srv/QueueInfoFetch.srv))
	Returns the meta information of a queue in the form of a [ros_queue_msgs/QueueInfo](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/msg/QueueInfo.msg). For example, you can fetch the queue info from the console with:

		rosservice call /<queue_server_name>/<queue_name>/getQueueInfo

* **`<queue_name>/getQueueSize`** ([ros_queue_msgs/FloatRequest](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/srv/FloatRequest.srv))
	Returns the current size of the queue in a float number.  For example, you can fetch the size of a given queue from the console with

		rosservice call /<queue_server_name>/<queue_name>/getQueueSize


#### Parameters
All the queue_server parameters are shown in the Config Files sections.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/etienn8/ros_queuing_system/issues).