#pragma once

#include <mutex>
#include <deque>

using std::deque;

template<typename TQueueElementType>
class ProtectedDeque
{
    private:
        std::mutex queue_manipulation_mutex_;
        deque<TQueueElementType> internal_queue_;
    public:
        inline virtual void push(TQueueElementType element_to_add)
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            internal_queue_.push_back(element_to_add);
        }

        virtual void pop()
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            internal_queue_.pop_front();
        }

        virtual TQueueElementType front()
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return internal_queue_.front();
        }

        virtual int size()
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return internal_queue_.size();
        }

        virtual void clear()
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            internal_queue_.clear();
        }

        virtual bool empty()
        {
            std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
            return internal_queue_.empty();
        }
};