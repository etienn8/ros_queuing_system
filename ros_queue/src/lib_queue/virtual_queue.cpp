#include <stdexcept>
#include "ros_queue/lib_queue/virtual_queue.hpp"

using std::invalid_argument;

inline float VirtualQueue::size()
{
    return queue_size_;
}

inline bool VirtualQueue::empty() 
{
    return queue_size_ == 0;
}

inline void VirtualQueue::push(const float& nb_element)
{
    if(nb_element < 0)
    {
        throw invalid_argument("Tried to push a negative number of elements in a virtual queue");
    }
    queue_size_ += nb_element;
}

void VirtualQueue::pop(const float& nb_element)
{
    if(nb_element < 0)
    {
        throw invalid_argument("Tried to pop a negative number of elements in a virtual queue");
    }

    if (nb_element > queue_size_)
    {
        queue_size_ = 0;
    }
    else
    {
        queue_size_ -= nb_element;
    }
}

inline void VirtualQueue::setSize(const float& new_size)
{
    if(new_size < 0)
    {
        throw invalid_argument("Tried to set the size of the queue to a negative value.");
    }
    queue_size_ = new_size;
}


inline float NVirtualQueue::size()
{
    return queue_size_;
}

inline bool NVirtualQueue::empty() 
{
    return queue_size_ == 0.0f;
}

inline void NVirtualQueue::push(const float& nb_element)
{
    if(nb_element < 0)
    {
        throw invalid_argument("Tried to push a negative number of elements in a virtual queue");
    }
    queue_size_ += nb_element;
}

inline void NVirtualQueue::pop(const float& nb_element)
{
    if(nb_element < 0)
    {
        throw invalid_argument("Tried to pop a negative number of elements in a virtual queue");
    }
    queue_size_ -= nb_element;
}

inline void NVirtualQueue::setSize(const float& new_size)
{
    queue_size_ = new_size;
}