#pragma once

#include <utility>

/**
 * @brief Container class that wraps an element with a user-defined size.
 * @tparam TQueueType Type of element with which a converted size will be linked to.
 */
template<typename TQueueElementType>
class ElementWithConvertedSize
{
    public:
        ElementWithConvertedSize(TQueueElementType&& element, unsigned int converted_size):element_(std::move(element)), converted_size_(converted_size){};
        TQueueElementType element_;
        unsigned int converted_size_=0;
};