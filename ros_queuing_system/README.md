# ros_queuing_system

## Overview

This the metapackage of the ros_queuing system. Its dependent on all the ros_queuing_system packages. Therefore, this metapackage helps to regroup all dependent system together.

**Keywords:** metapackage, queueing theory, ros_queuing_system

### License

The source code is released under a MIT license.

**Author: Étienne Villemure**
**Affiliation: MoreLab and [IntRoLab](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/Main_Page)**
**Maintainer: Étienne Villemure, etienne.villemure@usherbrooke.ca**

The ros_queuing_system package has been tested under [ROS] Noetic on Ubuntu 20.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation
### Building from Source
#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics).
- [queue_server](https://github.com/etienn8/ros_queuing_system/tree/main/queue_server) (Server of queues, included already in the ros_queuing_system repo).
- [ros_queue](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue) (Libraries of queues, included already in the ros_queuing_system repo).
- [ros_queue_msgs](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue_msgs) (ROS messages and services used as interface for the ros_queuing system, included already in the ros_queuing_system repo).
- [ros_queue_msgs](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue_msgs) (Implementation examples and dummy stochastic services to # ros_queuing_system

## Overview

This the metapackage of the ros_queuing system. Its dependent on all the ros_queuing_system packages. Therefore, this metapackage helps to regroup all dependent system together.

**Keywords:** metapackage, queueing theory, ros_queuing_system

### License

The source code is released under a MIT license.

**Author: Étienne Villemure**
**Affiliation: MoreLab and [IntRoLab](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/index.php/Main_Page)**
**Maintainer: Étienne Villemure, etienne.villemure@usherbrooke.ca**

The ros_queuing_system package has been tested under [ROS] Noetic on Ubuntu 20.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation
### Building from Source
#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics).
- [queue_server](https://github.com/etienn8/ros_queuing_system/tree/main/queue_server) (Server of queues, included already in the ros_queuing_system repo).
- [ros_queue](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue) (Libraries of queues, included already in the ros_queuing_system repo).
- [ros_queue_msgs](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue_msgs) (ROS messages and services used as interface for the ros_queuing system, included already in the ros_queuing_system repo).
- [ros_queue_msgs](https://github.com/etienn8/ros_queuing_system/tree/main/ros_queue_msgs) (Implementation examples and dummy stochastic services to test the system, included already in the ros_queuing_system repo).
- [rosparam_utils](https://github.com/etienn8/rosparam_utils) (Tools to fetch rosparams more easily. Not directly dependent but some dependent packages need it).
##### Optional
- [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/#)(Tools to help build all the packages in parallel and to test the system)
	- Follow their [installation](https://catkin-tools.readthedocs.io/en/latest/installing.html) and [initialization](https://catkin-tools.readthedocs.io/en/latest/quick_start.html) guide before building this package.


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
This package is mainly used to install and build all the necessary packages to get a functionnal ROS queuing system.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/etienn8/ros_queuing_system/issues).
test the system, included already in the ros_queuing_system repo).
- [rosparam_utils](https://github.com/etienn8/rosparam_utils) (Tools to fetch rosparams more easily. Not directly dependent but some dependent packages need it).
##### Optional
- [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/#)(Tools to help build all the packages in parallel and to test the system)
	- Follow their [installation](https://catkin-tools.readthedocs.io/en/latest/installing.html) and [initialization](https://catkin-tools.readthedocs.io/en/latest/quick_start.html) guide before building this package.


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
This package is mainly used to install and build all the necessary packages to get a functionnal ROS queuing system.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/etienn8/ros_queuing_system/issues).
