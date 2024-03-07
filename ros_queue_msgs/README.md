# ros_queue_msgs

## Overview

This package describes all the ROS messages and services used by the ros_queuing_system. Therefore, it contains all the interfaces for other packages to interact with the queuing system. It aims at reducing the build dependencies since only the interface should be known.

**Keywords:** rosmsg, rosservice, ros_queuing_system

### License

The source code is released under a MIT license.

**Author: Étienne Villemure**
**Affiliation: [MoreLab](https://morelab.ca/) and [IntRoLab](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/Main_Page)**
**Maintainer: Étienne Villemure, etienne.villemure@usherbrooke.ca**

The ros_queue_msgs package has been tested under [ROS] Noetic on Ubuntu 20.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- std_msgs,
- [genmsg](http://wiki.ros.org/genmsg) (Library to generate some ROS message),
- [actionlib_msgs](http://wiki.ros.org/actionlib_msgs) (Common messages definitions of the actionlib library)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/etienn8/rosparam_utils.git
	git clone https://github.com/etienn8/ros_queuing_system.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

If you're using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html), you could use `catkin build` command instead of `catkin_make`.

## Messages definitions
* **`ManualByteTransmitSize.msg`**
	Structure that contains an integer indicating how many bytes a real queue should manually transfer.
* **`PotentialTransmissionVectorSet.msg`**
	Structure that contains an array of TransmissionVector.msg messages that represents a set of potential actions.
* **`QueueInfo.msg`**
	Structure  that contains metainformation about the queues like their names.
* **`QueueIntElement.msg`**
	Example of a queue element that could be stored in a [ROSQueue](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue). This implementation is message that only contains an int.
* **`QueueServerState .msg`**
	Metainformation about the [queue_server](https://github.com/etienn8/ros_queuing_system/tree/main/queue_server) and all the state of its queues given by an array of `ros_queue_msgs/QueueState`. 
* **`QueueStats.msg`**
	Contains some statistics of the time average metrics of a queue. It includes the mean size of the queue over all frames and the time average or the arrivals and departures. The real departure represents the real data that could be transmitted if the queue was smaller than the transmission $`b(t)`$: $`\hat{b(t)}=min(b(t),Q(t))`$ for real queues and $`\hat{b}=min(b(t),q(t)+(a))`$ for virtual queues. Also, for real queues, it also sends the converted_remaining_mean that corresponds to the amount of data that could be sent because the remaining data that could be sent is smaller larger than the number of bytes in the front element in the queue. For virtual queues, it also sends the time average of the changes.
* **`QueueServerStats.msg`**
	Contains an array of QueueStats of all real queues from a queue server.
* **`QueueState.msg`**
	Represent the state of the queue and its identifier. It's mainly to get its name and its size.
* **`QueueTransmitTemplate.msg`**
	Its an array of element that could be stored in queues. Its message template that should be copied where its array type should be changed to reflect the stored elements in a real queue. In this implementation, the type of the array is  `ros_queue_msgs/QueueIntElement`. Changing this value would required to change some include files in [ros_queue](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue) and to recompile the code.
* **`TransmissionVector.msg`**
	Contains an array with a size that represents the number of queues in a system and where their boloean values indicate if they could transmit or not.
* **`VirtualQueueChanges.msg`**
	Manual changes on the arrival and the departures that should be manually applied to a specified(based on name of the queue) virtual queue.
* **`VirtualQueueChangesList.msg`**
	List of VirtualQueueChanges.msg that represents the all the changes that need to be applied to the list queues of a queue server.
* **`States.msg`**
	Represent the state of the the whole system that the queuing system is interact with. Its serves as template where states (ex: temperature, position,...) that are useful to evaluate the change of queues (mainly the virtual queues values and how much data that could be transmitted via the real queues). 
	
## Services definitions

* **`ByteSizeRequest.srv`**
	Empty request service call that returns an int. It's mainly used to return a number of bytes. 
* **`FloatRequest.srv`**
	Empty request service call that returns a float value. 
* **`ConversionTemplateService.srv`**
	Service that takes a `ros_queue_msgs/QueueIntElement` array. It returns an array of the int that represent a cost for each of the element in the input array. This message is designed to be used as a template to copy and where to replace the input array type (ex:`QueueIntElement`) for the type of the message that is stored in a real queue.
* **`QueueinfoFetch.srv`**
	Empty request service call that returns the `ros_queue_msgs/QueueInfo`. It's used to fetch the meta information of a queue. 
* **`QueueServerStateFetch.srv`**
	Empty request service call that returns a `ros_queue_msgs /QueueServerState`. It's mainly used to fetch the current state of a [queue_server](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue)
* **`QueueStatesPredictions.srv`**
	Service call that takes arbitrary `ros_queue_msgs/States` and it returns a prediction in the form of an int. This is used for predicting the value of a metric based on the current state of the system. This definition is mainly designed to be copied to change the definition of the `ros_queue_msgs/States` for any `States`. However, doing so will required to recompile the code and change the included files in the queue_controller. *Subject to change since it should return a float*. 
* **`MetricTransmissionVectorPredictions.srv`**
	Structure that contains a action set of TransmissionVectorSet.msg and that returns a float array of predictions for each action in the set. The output float index should match its corresponding action index.
* **`PotentialTransmissionVectorSpaceFetch.srv`**
	Empty request structure that returns a potential set of type PotentialTransmissionVectorSet.msg. It's usualy used to get all the potential actions for a current time frame.
	
## Action definitions
* **`TransmissionVector.action`**
	Actionlib action that takes a ros_queue_msgs/TransmissionVector as it's goal and returns a sucess flag as a result (linked to if all transmission has occured). No feedback was implemented.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/etienn8/ros_queuing_system/issues).
