#pragma once

#include <queue>

using std::queue;

namespace queue_utils
{
    template<typename T>    
    void concatenate_queue(queue<T> &receiving_queue, queue<T> queue_to_add)
    {
        while(!queue_to_add.empty())
        {
            receiving_queue.push(queue_to_add.front());
            queue_to_add.pop();
        }
    }
}