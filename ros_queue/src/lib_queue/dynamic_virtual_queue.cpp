#include <stdexcept>

#include "ros_queue/lib_queue/dynamic_virtual_queue.hpp"
#include "ros_queue/lib_queue/queue_exception.hpp"

using std::invalid_argument;

int InConVirtualQueue::getSize()
{
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    return internal_queue_.size();
}

int InConVirtualQueue::evaluate()
{
    const int arrival = arrival_prediction();
    const int departure = transmission_prediction();

    if (arrival < 0)
    {
        throw NegativeArrivalPredictionException("");
    }
    if (departure < 0)
    {
        throw NegativeDeparturePredictionException("");
    }

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

bool InConVirtualQueue::update(VirtualQueue arriving_elements, const int nb_departing_elements)
{   
    const int arrival = arriving_elements.size();
    
    if(nb_departing_elements < 0)
    {
        throw invalid_argument("Tried to remove a negative number of elements from the queue.");
    }

    // Protect access to queue
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

    const int current_size = internal_queue_.size();

    // Queue dynamic
    int new_size = (current_size > nb_departing_elements) ? current_size - nb_departing_elements + arrival : arrival; 

    bool overflowed = false;
    if (new_size > max_queue_size_)
    {
        new_size = max_queue_size_;
        overflowed = true;
    }

    internal_queue_.setSize(new_size);

    return !overflowed;
}

bool InConVirtualQueue::update(const int nb_arriving_elements, const int nb_departing_elements)
{
    if(nb_departing_elements < 0)
    {
        throw invalid_argument("Tried to remove a negative number of elements from the queue.");
    }

    if(nb_arriving_elements < 0)
    {
        throw invalid_argument("Tried to add a negative number of elements from the queue.");
    }

    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);

    const int current_size = internal_queue_.size();

    // Queue dynamic
    int new_size = (current_size > nb_departing_elements) ? current_size - nb_departing_elements + nb_arriving_elements : nb_arriving_elements; 

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

int EqConVirtualQueue::evaluate()
{
    const int arrival = arrival_prediction();
    const int departure = transmission_prediction();

    if (arrival < 0)
    {
        throw NegativeArrivalPredictionException("");
    }
    if (departure < 0)
    {
        throw NegativeDeparturePredictionException("");
    }

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

bool EqConVirtualQueue::update(NVirtualQueue arriving_elements, const int nb_departing_elements)
{
    const int arrival =  arriving_elements.size();

    if(nb_departing_elements < 0)
    {
        throw invalid_argument("Tried to remove a negative number of elements from the queue.");
    }
    
    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    const int current_size = internal_queue_.size();

    // Dynamics of a virtual queue that constrain a  time average value at zero
    int new_size = current_size - nb_departing_elements + arrival; 

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

bool EqConVirtualQueue::update(const int nb_arriving_elements, const int nb_departing_elements)
{   
    if(nb_departing_elements < 0)
    {
        throw invalid_argument("Tried to remove a negative number of elements from the queue.");
    }

    if(nb_arriving_elements < 0)
    {
        throw invalid_argument("Tried to add a negative number of elements from the queue.");
    }

    std::lock_guard<std::mutex> lock(queue_manipulation_mutex_);
    const int current_size = internal_queue_.size();

    // Dynamics of a virtual queue that constrain a  time average value at zero
    int new_size = current_size - nb_departing_elements + nb_arriving_elements; 

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