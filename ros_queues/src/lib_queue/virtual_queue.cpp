#include "ros_queues/lib_queue/virtual_queue.hpp"

inline int VirtualQueue::size()
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    return queue_size_;
}

inline bool VirtualQueue::empty() 
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    return queue_size_ == 0;
}

inline void VirtualQueue::push(const int& nb_element)
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    queue_size_ += nb_element;
}

void VirtualQueue::pop(const int& nb_element)
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    if (nb_element > queue_size_)
    {
        queue_size_ = 0;
    }
    else
    {
        queue_size_ -= nb_element;
    }
}

inline void VirtualQueue::setSize(const int& new_size)
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    queue_size_ = new_size;
}


inline int NVirtualQueue::size()
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    return queue_size_;
}

inline bool NVirtualQueue::empty() 
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    return queue_size_ == 0;
}

inline void NVirtualQueue::push(const int& nb_element)
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    queue_size_ += nb_element;
}

inline void NVirtualQueue::pop(const int& nb_element)
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    queue_size_ -= nb_element;
}

inline void NVirtualQueue::setSize(const int& new_size)
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    queue_size_ = new_size;
}