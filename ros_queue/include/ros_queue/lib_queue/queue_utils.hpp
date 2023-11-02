#pragma once

namespace queue_utils
{
    template <typename TQueueElementType>
    void concatenate_queues(deque<TQueueElementType>& queue_to_expand, const deque<TQueueElementType>& queue_to_copy)
    {
        for(typename deque<TQueueElementType>::const_iterator it = queue_to_copy.begin(); it != queue_to_copy.end(); ++it)
        {
            queue_to_expand.push_back(*it);
        }
    }
}