#pragma once

#include "ros_queue_msgs/MetricTransmissionVectorPredictions.h"
#include "ros_queue_msgs/TransmissionVector.h"
#include "ros_queue_msgs/PotentialTransmissionVectorSet.h"
#include "ros_queue_msgs/PotentialTransmissionVectorSpaceFetch.h"
#include "ros_queue_msgs/TransmissionVectorAction.h"


typedef ros_queue_msgs::MetricTransmissionVectorPredictions METRIC_CONTROL_PREDICTION_SRV;
typedef ros_queue_msgs::PotentialTransmissionVectorSet POTENTIAL_ACTION_SET_MSG;
typedef ros_queue_msgs::PotentialTransmissionVectorSpaceFetch POTENTIAL_ACTION_SET_SRV;
typedef ros_queue_msgs::TransmissionVectorAction ACTION_LIB_OUTPUT_TYPE;