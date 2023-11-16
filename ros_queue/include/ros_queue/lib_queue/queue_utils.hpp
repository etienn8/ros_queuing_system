#pragma once

#include <utility>

namespace queue_utils
{
    template <typename TQueueElementType>
    
    /**
     * @brief Function that moves the content of the queue_to_move at the end of the queue_to_expand. It uses the std::move to move data from a queue to another so the queue_to_move could not usable after this function.
     */
    void concatenate_queues(deque<TQueueElementType>& queue_to_expand, deque<TQueueElementType>&& queue_to_move)
    {
        for(typename deque<TQueueElementType>::const_iterator it = queue_to_move.begin(); it != queue_to_move.end(); ++it)
        {
            queue_to_expand.push_back(std::move(*it));
        }
    }
}