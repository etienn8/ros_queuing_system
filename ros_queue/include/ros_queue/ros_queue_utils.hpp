#pragma once

/**
 * @brief Struct to help the compiler resolving the type of the vector queue_elements contained in a TROSMsgType.
 * @tparam TROSMsgType Type of a ROS message that contains an array name queue_elements
*/
template <typename TROSMsgType>
struct QueueElementTrait {
  using ElementType = typename TROSMsgType::_queue_elements_type::value_type;
};
