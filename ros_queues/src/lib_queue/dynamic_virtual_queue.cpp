#include "ros_queues/lib_queue/dynamic_virtual_queue.hpp"

int InConVirtualQueue::getSize()
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    return internal_queue_.size();
}

int InConVirtualQueue::getMemSize()
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    return sizeof(internal_queue_.size());
}

int InConVirtualQueue::evaluate()
{
    const int arrival = arrival_prediction();
    const int departure = transmission_prediction();

    //Protect access to queue
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

    const int current_size = internal_queue_.size();

    // Queue dynamic
    int new_size = (current_size > departure) ? current_size - departure + arrival : arrival; 

    if (new_size > max_queue_size_)
    {
        new_size = max_queue_size_;
    }
    
    return new_size;
}

bool InConVirtualQueue::update(VirtualQueue arriving_elements, const unsigned int departure)
{   
    const int arrival = arriving_elements.size();

    // Protect access to queue
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

    const int current_size = internal_queue_.size();

    // Queue dynamic
    int new_size = (current_size > departure) ? current_size - departure + arrival : arrival; 

    bool overflowed = false;
    if (new_size > max_queue_size_)
    {
        new_size = max_queue_size_;
        overflowed = true;
    }

    internal_queue_.setSize(new_size);

    return !overflowed;
}

bool InConVirtualQueue::udpate(const int arrival, const unsigned int departure)
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

    const int current_size = internal_queue_.size();

    // Queue dynamic
    int new_size = (current_size > departure) ? current_size - departure + arrival : arrival; 

    bool overflowed = false;
    if (new_size > max_queue_size_)
    {
        new_size = max_queue_size_;
        overflowed = true;
    }

    internal_queue_.setSize(new_size);
    return !overflowed;
}


int InConVirtualQueue::arrival_prediction()
{
    return 0;
}

int InConVirtualQueue::transmission_prediction()
{
    return 0;
}


int EqConVirtualQueue::getSize()
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    return internal_queue_.size();
}

int EqConVirtualQueue::getMemSize()
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    return sizeof(internal_queue_.size());
}

int EqConVirtualQueue::evaluate()
{
    const int arrival = arrival_prediction();
    const int departure = transmission_prediction();

    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    const int current_size = internal_queue_.size();

    // Dynamics of a virtual queue that constrain a  time average value at zero
    int new_size = current_size - departure + arrival; 

    if (new_size > max_queue_size_)
    {
        new_size = max_queue_size_;
    }
    else if (new_size < -max_queue_size_)
    {
        new_size = -max_queue_size_;
    }
    
    return new_size;
}

bool EqConVirtualQueue::update(NVirtualQueue arriving_elements, const unsigned int departure)
{
    const int arrival =  arriving_elements.size();

    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    const int current_size = internal_queue_.size();

    // Dynamics of a virtual queue that constrain a  time average value at zero
    int new_size = current_size - departure + arrival; 

    bool overflowed = false;
    if (new_size > max_queue_size_)
    {
        new_size = max_queue_size_;
        overflowed = true;
    }
    else if (new_size < -max_queue_size_)
    {
        new_size = -max_queue_size_;
        overflowed = true;
    }

    internal_queue_.setSize(new_size);
    return !overflowed;
}

bool EqConVirtualQueue::udpate(const int arrival, const unsigned int departure)
{   
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    const int current_size = internal_queue_.size();

    // Dynamics of a virtual queue that constrain a  time average value at zero
    int new_size = current_size - departure + arrival; 

    bool overflowed = false;
    if (new_size > max_queue_size_)
    {
        new_size = max_queue_size_;
        overflowed = true;
    }
    else if (new_size < -max_queue_size_)
    {
        new_size = -max_queue_size_;
        overflowed = true;
    }

    internal_queue_.setSize(new_size);
    return !overflowed;
}

int EqConVirtualQueue::arrival_prediction()
{
    return 0;
}

int EqConVirtualQueue::transmission_prediction()
{
    return 0;
}