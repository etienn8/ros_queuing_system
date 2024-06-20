# ros_queue

## Overview
Package that contains libraries of queue-type data structures. These queues are based on the work of Neely[1]. Two major types of queues are defined: virtual queues and real queues.

A version of those queues wrapped in ROS services and topics are provided to let the queues store ROS messages and services to indicate the size, the increase and the decrease of the queues.

Its best utilized with the [queue_server](https://github.com/etienn8/ros_queuing_system/tree/main/queue_server) package that has a node to instantiate those queues and holds them.

**Keywords:** queuing theory, queues

### lib_queue overview

The core C++ library in this package, lib_queue, contains definitions of real queues that can store any type of data structure as elements (using std::deque as its core). The library wraps these internal queues in a dynamic queue, which adds interfaces to update the queue. The queues are updated based on the following equation:$$q(t+1) = max[q(t) - b(t), 0] + a(t)$$ where $t$ represent the time at which the equation is evaluated, $q$ represents the queue size, $a$ represents the number of incoming elements in the queue (the increase of the queue size) and $b$ represents the number of transmitted elements (decrease of the queue size).

The virtual queues in the *lib_queue* library only contain a size (which is a float value) since they don't actually hold data. They are governed by the same dynamic equation as the real queues except for the equality constraint queue that can have a negative size.

The library uses C++ [move semantics](https://www.cprogramming.com/c++11/rvalue-references-and-move-semantics-in-c++11.html) to reduce the memory usage and computation when moving queue elements around.

## ros_queue overview
The *ros_queue* library wraps the *lib_queue* library with ROS services and topics to interact with the queues. 

For the real queues, it mainly provides a ROS subscriber that stores its incoming topic messages in the queue, provides an interface to transmit its data in another topic and provides interfaces to know their state.

For the virtual queues, it mainly provides ROS services to evaluate by how much the virtual queue should increase and decrease and provides interfaces to know their states. 
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
- [topic_tools](http://wiki.ros.org/topic_tools) (Contains the Shapeshifter class that allows to store ROS message types that are only know at runtime),
-  [rostest](http://wiki.ros.org/rostest) (ROS test library that allow automated testing of the *lib_queue* and *ros_queue libraries*),
- [std::deque](https://en.cppreference.com/w/cpp/container/deque)(Standard library double ended-queues),
- std::utility (Standard library that permits move semantics),
- std::mutex (Standard library to manage concurrency)
- std::except (Standard library for throwing exceptions)

	sudo rosdep install --from-paths src
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

### Unit Tests

With [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html), run the unit tests of all packages with

	catkin test

If you want to only run the tests of this package:

	roscd ros_queue
	catkin test --this

## Usage
Since this package only contains libraries and no packages on its own, some examples are shown below to present the queues that can be created. 

### lib_queue

#### DynamicQueue
[Header with documentation.](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/include/ros_queue/lib_queue/dynamic_queue.hpp)

It's a real queue that is modified with an update function that follows the dynamics presented in the overview and that has a maximum size. Internally, it uses std::deque where the type of data held by the queue is specified as a template argument. The most general form to instantiate this type of queue is: 
```
DynamicQueue<TypeOfTheElementToStoreInTheQueue, TypeOfTheArgumentGivenToPredictTheChangeInsize[void if not specified]> queue(queue_max_size);
```

Here's an example on how to create a Dynamic queue that contains int values.
```
#include "ros_queue/lib_queue/dynamic_queue.hpp"

DynamicQueue<int> q(10); // Creates a queue with int values and that has a maximum size of 10 int.
```

To update the queue, the `update(deque<TypeOfTheElementToStoreInTheQueue> queue, int number_of_departing_elements)` method is used:
```
...
deque<int> arrival_queue = {1, 2, 3, 4, 5};

q.update(arrival_queue, 0);  // Copies the arrival queue in the queue and transmit 0 data.
q.getSize(); //Gets the size of the queue. Should be 5 in this case

q.update(std::move(arrival_queue), 0);  // Moves the arrival_queue in the queue and transmit 1 data. More efficient since no copie occurs. However, arrival_queue might become unusable. 
q.getSize(); //Gets the size of the queue. Should be 9 in this case since we added 5 more data and transmited one.

deque<int> internal_queue = q.getInternalQueue(); // Gets a copy of the the internal queue.
```

A feature has been integrated to evaluate how the queue will change based on a specified arrival and departure without actually modifying the queue. The user should create a new class that inherits from DynamicQueue and redefine its methods `int arrival_prediction()` and `int transmission_prediction()`. Also the same method names could redefined but with an argument specified by the template `int arrival_prediction(const TypeOfTheArgumentGivenToPredictTheChangeInsize& states)`. Then you call the `evaluate()` method or `evaluate(states)`. Example:

```
#include "ros_queue/lib_queue/dynamic_queue.hpp"

class IntQueueWithFloatPrediction: DynamicQueue<int, float> // Inherit from a int Dynamic queue with its evaluation argument as a float value.
{
	IntQueueWithFloatPrediction(int max_queue_size): DynamicQueue<int, float>(max_queue_size) {};
	
	virtual int arrival_prediction(float states) override
	{
		return static_cast<int>(states)+1.0f;
	};
	
	virtual int transmission_prediction(float states) override
	{
		return static_cast<int>(states);
	};
}

IntQueueWithFloatPrediction q(10);

int predicted_size = q.evaluate(1.0f); //Should be 3.0 based on the dynamic queue equations.
q.getSize(); //Gets the size of the queue. Should be 0.0 since the evaluate method doesn't modify the queue.
```

#### DynamicConvertedQueue
[Header with documentation.](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/include/ros_queue/lib_queue/dynamic_converted_queue.hpp)

It's very similar to DynamicQueue where its holds real data structure and uses also an update and evaluate function but where the size of the queue (evaluated by `getSize()`) could be different then the number of elements in the queue. For example, for a given system, the queue might store elements in the form or std::vector that could have different sizes. In that given system, it might be more useful to compute the size of the queue in terms of bytes or as the sum of arrays' sizes.

To do so, all incoming "queue elements" are coupled with their converted size in new objects that are in turn stored in the real queue. To override the base conversion (queue size = number of elements), the developer needs to create a new class, inherit from DynamicConvertedQueue and override the `generateConvertedQueue` method. Here's an example where the size of the queue is the number of bytes in all its vector<int> element:
```
#include "ros_queue/lib_queue/dynamic_converted_queue.hpp"
#include <vector>

class IntVectorQueueWithByteSize: public DynamicConvertedQueue<std::vector<int>>
{
	public:
		IntVectorQueueWithByteSize(int max_queue_size): DynamicConvertedQueue<std::vector<int>>(max_queue_size) {};

	protected:
		virtual void generateConvertedQueue(deque<std::vector<int>>&& arriving_queue, deque<ElementWithConvertedSize<std::vector<int>>>& converted_queue) override
			{
				for(deque<std::vector<int>>::iterator it = arriving_queue.begin(); it != arriving_queue.end(); ++it)
					{
					int converted_size = it->size() * sizeof(int);

					ElementWithConvertedSize<std::vector<int>> convertedElement(std::move(*it), converted_size);
					converted_queue.push_back(std::move(convertedElement));
					}
				}
};

```
The max_queue_size in this case is specified in the converted size.

Also, it's important that the converted size is not negative and non-zero. Otherwise, an exception will be thrown.


To update the queue, two methods exists:
```
...
IntVectorQueueWithByteSize q(200); //The converted queue class defined in the example above with a max size of 200 bytes.

// Create dummy input data
std::vector<int> int_vector = {1, 2, 3, 4, 5};

std::deque<std::vector<int>> arriving_queue;
arriving_queue.push_back(int_vector);
arriving_queue.push_back(int_vector);

q.update(arriving_queue,0); //Adds a queue with two vectors with 5 int each and transmit 0 vector (0 queue element).
q.update(arriving_queue, 2);//Adds a queue with two vectors with 5 int each and removes 2 vectors (2 queue elements)

q.updateInConvertedSize(arriving_queue, 25); // Adds a queue with two vectors with 5 int each and tries to remove up to 25 converted size (in this case, bytes). Since each vector contains 5 int (20 bytes), only one vector will be transmitted.

q.update(updateInConvertedSize, 19); // This will not transmit anything since the size of the element to send is bigger than what is asked to be transmitted. 
```

#### InConVirtualQueue
[Header with documentation.](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/include/ros_queue/lib_queue/dynamic_virtual_queue.hpp)
Virtual queues don't hold data like a real queues. They just use a float as a size and use the queue dynamic formula to be analyzed and used like a queue. Based on the work of Neely [1], virtual queues are often used to represent a metric that is under a constraint. When used as time average inequality constraint, the virtual queues size can't have a negative value (like a real queue). This queue type is implemented in this library as InConVirtualQueue (Inequality constraint Virtual Queue). Here's a simple example on how to instantiate and update them:

```
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"

InConVirtualQueue vq(10.0f); // Create a virtual queue with a size of 10.0

vq.update(4.0f-3.0f);  // Increase the queue by 1.0f. 
vq.getSize();   // Returns the size of the queue. Should be 1.0f
```

#### EqConVirtualQueue
[Header with documentation.](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/include/ros_queue/lib_queue/dynamic_virtual_queue.hpp)
Based on the work of Neely [1], some virtual queues could be used to represent a metric that is under a time average equality constraint, the size of the queue can go below zero (so its only capped by it's maximum size). Thus the name of this implementation is EqConVirtualQueue (Equality Constraint Virtual Queue). Here's an example: 
```
#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"

EqConVirtualQueue vq(10.0f); // Create a virtual queue with a size of 10.0

vq.update(-1.0f);  // Decrease the queue by 1.0f. 
vq.getSize();   // Returns the size of the queue. Should be -1.0f where a InConVirtualQueue would have return 0.0f.
```

### Other notes for the lib_queue
The majors differences between the implementation of the real queues and the virtuals queues are:
- Their size are a in float values.
- They don't use templates and they all inherit from a common `IDynamicVirtualQueue` class. So all virtual queues can be updated through a cast to this base class.
- Their update method only has one argument that represent the change in the queue size.
 
More examples can be found in the [dynamic queues test file](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/test/dynamic_queues_test.cpp)for the real queues and in the [virtual queues test file](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/test/virtual_queues_test.cpp) for the virtual queues if needed.

### ros_queue
Its a library that wraps the *lib_queue* queues with ROS services and topics to interact with the queues. 

#### Common interfaces
All the classes of the *ros_queue* library inherit from a [ROSQueueCommonInterfaces](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/include/ros_queue/ros_queue_common_interfaces.hpp) class. 

Passed by the constructor of all queues, it holds a ROS NodeHandle and the queue meta information. The latter is held in a [ros_queue_msgs::QueueInfo](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/msg/QueueInfo.msg) ROS message so it could be easily send through a ROS interface. The `queue_name` in the QueueInfo provided should be non empty otherwise an exception will be thrown.

Based on the `queue_name` provided in its `QueueInfo`, the queue will advertise two services:
- `<queue_name>/getQueueSize`: ([ros_queue_msgs/FloatRequest](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/srv/FloatRequest.srv)) 
	Returns the size of the queue in a float size (so it could be used both by the virtual and real queues). The request is empty.
	
- `<queue_name>/getQueueInfo`: ([ros_queue_msgs/QueueInfoFetch](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/srv/QueueInfoFetch.srv))
	Returns the `QueueInfo` object held by the queue that indicates its metadata. The request is empty.
	
#### ROSvirtualQueue
[Header with documentation.](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/include/ros_queue/ros_virtual_queue.hpp)
Its the class that wraps the `InConVirtualQueue` and the `EqConVirtualQueue` in ROS services or user-defined functions. The used of `InConVirtualQueue` or `EqConVirtualQueue` is specified with a template argument.

The main interfaces to be defined are the increase of the queue and its decrease when the `update()` is called. An internal structure of the class specifies all the possible arguments and are to the constructor:

```
struct InterfacesArgs
{
	float (*arrival_evaluation_fptr)() = nullptr;
	string arrival_evaluation_service_name = "";
	
	
	float (*departure_evaluation_fptr)() = nullptr;
	string departure_evaluation_service_name = "";
};
```
In other words, the arrival (increase) and the departure (decrease) could be either evaluated by a user-defined function given by a function pointer or a service name that calls a [ros_queue_msgs/FloatRequest](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/srv/FloatRequest.srv) service.

The user-defined function pointer should have the following signature: `float (*arrival_evaluation_fptr_)() = nullptr; float (*departure_evaluation_fptr_)() = nullptr;`.

For the arrival and departure, either the service name or the function pointer should be defined but the arrival interface doesn't need be the same interface as the departure. If a function pointer and a service name are defined, the user-defined function will be used and the service client won't be created.

Here's some examples on how to instantiate a ROSVirtualQueue:
```
#include "ros_queue/ros_virtual_queue.hpp"


namespace metric_computation
{
    float mocked_arrival = 3.0;
    float mocked_departure = 2.0;

    float getMockedArrival()
    {
        return mocked_arrival;
    }

    float getMockedDeparture()
    {
        return mocked_departure;
    }
}

ros::NodeHandle nh;

ros_queue_msgs::QueueInfo queue_info;
queue_info.queue_name = "Queue0";

// Creates a ROS virtual queue with a max size of 10.0 and where its update is based on two function pointers.
ROSVirtualQueue<InConVirtualQueue> vq0(10.0f, std::move(queue_info), nh,
										(ROSVirtualQueue<InConVirtualQueue>::InterfacesArgs){
										.arrival_evaluation_fptr = metric_computation::getMockedArrival,
										.departure_evaluation_fptr = metric_computation::getMockedDeparture});

vq0.update(); // Calls internaly the function pointers to udate the change in the virtual queue.
vq0.getSize(); // Returns the size of the queue. Should be 1.0f in this case.

ros_queue_msgs::QueueInfo queue_info1;
queue_info1.queue_name = "Queue1";

// Create a ROS virtual queue with a max size of 15.0 and where its arrival is computed by the "external_arrival_service" service and its departure is computed by a function pointer.
ROSVirtualQueue<InConVirtualQueue> vq1(10.0f, std::move(queue_info1), nh,
										(ROSVirtualQueue<InConVirtualQueue>::InterfacesArgs){
										.arrival_evaluation_service_name = "external_arrival_service",
										.departure_evaluation_fptr = metric_computation::getMockedDeparture});

vq1.update(); // Increase the queue on based the arrival evaluation service response and decrease by an amout provided by the departure function pointer.
```

#### ROSByteConvertedQueue 
[Header with documentation](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/include/ros_queue/ros_byte_converted_queue.hpp).
The `ROSByteConvertedQueue` is a specific type of `DynamicConvertedQueue` where its stored messages are [topic_tools::ShapeShifter::ConstPtr](http://docs.ros.org/en/indigo/api/topic_tools/html/classtopic__tools_1_1ShapeShifter.html) and the converted size of the queue is the sum of the size of all its stored messages in bytes. It subscribes to a ROS topic for its arrival and adds the arriving message in the queue. It transmits the same message types through another transmission topic. 

The `ShapeShifter` object removes the need for a ROS subscriber to know at compile time the type of the ROS message it will be receiving. Once the first message is received, a publisher is created from the message type known by `ShapeShifter` that will be used for transmission. In other words, no transmission topic will be advertised as long as no message has been received. The main constraint of using `ShapeShifter` is that, since the message type is not known at compile time, no interpretation of the data can be done so not converted function could be specified. However, thanks to the `ShapeShifter`'s method `size()`, the size in bytes of the message could be computed whatever the real ROS message. 

The `ROSByteConvertedQueue` is configured at its construction mainly through its interface arguments that should all be defined.
All three interface arguments are given in the example below:

```
#include "ros_queue/ros_byte_converted_queue.hpp"

// Creates a converted queue in bytes with a maximum size of 200 bytes that takes in input the messages from the arrival_topic_name, transmits over the transmission_topic_name and uses the transmission_evaluation_service_name to evaluate how much data to transmit based on this external service.

ROSByteConvertedQueue q0(200, std::move(queue_info_f), nh_f,
						(struct ROSByteConvertedQueue::InterfacesArgs){
							.arrival_topic_name = "arrival_topic",
							.transmission_topic_name = "transmission_topic",
							.transmission_evaluation_service_name = transmission_topic_name_f

})
```
The `arrival_topic_name` specifies the topic from which message will be stored in the queue, the `transmission_topic_name` specifies the topic name to publish the stored message whenever the update ask to transmit and the `transmission_evaluation_service_name` is a service name used in the `transmitBasedOnQoS()` method which indicate a number of bytes to transmit.

In sum, the queue grows whenever a message is received on the `arrival_topic_name` and it decreases whenever a call is made to its transmission methods:
```
...
q0.transmit(5); // Transmit the first elements of the deque.
q0.transmitBasedOnQoS(); // Calls the transmission_evaluation_service_name to know how much bytes can be transmitted and transmit that much data if possible.
```


##### Services:
- `<transmission_evaluation_service_name>`: ([ros_queue_msgs/ByteSizeRequest](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue_msgs/srv/ByteSizeRequest.srv)) 
	Service that returns an int of how many bytes that could be sent from the queue at the moment of the call. The request is empty.

###### Topics: 
Subscriber:
- `<arrival_topic_name>`: (Can be any ROS messages)
	The messages from this topic will be stored in the queue. After the first reception of the message, a publisher will be created.
	
- `nb_bytes_to_transmit`: (std_msgs/Int32)
	The queue will try to transmit the number of bytes specified by the integer received in this incoming message.

Publisher:
- `<transmission_topic_name>`: (Type of the first message received at the <arrival_topic_name>)
	Output of the queue updates whenever there is a transmission.

#### Other notes for the ros_queue
The [ROSQueue](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/include/ros_queue/ros_queue.hpp) and the [ROSConvertedQueue](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/include/ros_queue/ros_converted_queue.hpp) are two classes that are not directly documented in this readme since they are heavy to configure. This is mainly due by the fact that they still used predictions which is a deprecated feature and because they use templates. They are similar to ROSVirtualQueue and ROSByteConvertedQueue; however, through their templates, they require knowledge of the types of stored elements (ROS message definition) at compile time. Also, those two classes lack the transmission evaluation function that allows them to transmit data based on the transmission channel state. They should be integrated to be used with the [queue_server](https://github.com/etienn8/ros_queuing_system/tree/main/queue_server).


More implementation examples could be found for the ros_queue library (including examples for the ROSQueue and the ROSConverteQueue) through the [automated test file](https://github.com/etienn8/ros_queuing_system/blob/main/ros_queue/test/ros_queue_test.cpp).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/etienn8/ros_queuing_system/issues).
# References
[1] M. J. Neely, *Stochastic Network Optimization with Application to Communication and Queueing Systems*, 1st ed. Cham: Springer Cham, 2022.
