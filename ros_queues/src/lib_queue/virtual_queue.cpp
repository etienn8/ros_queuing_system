#include "ros_queues/lib_queue/virtual_queue.hpp"

inline int VirtualQueue::size()
{
    return queue_size_;
}

inline bool VirtualQueue::empty() 
{
    return queue_size_ == 0;
}

inline void VirtualQueue::push(const int& nb_element)
{
    queue_size_ += nb_element;
}

void VirtualQueue::pop(const int& nb_element)
{
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
    queue_size_ = new_size;
}


inline int NVirtualQueue::size()
{
    return queue_size_;
}

inline bool NVirtualQueue::empty() 
{
    return queue_size_ == 0;
}

inline void NVirtualQueue::push(const int& nb_element)
{
    queue_size_ += nb_element;
}

inline void NVirtualQueue::pop(const int& nb_element)
{
    queue_size_ -= nb_element;
}

inline void NVirtualQueue::setSize(const int& new_size)
{
    queue_size_ = new_size;
}