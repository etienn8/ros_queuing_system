#include "ros_queue_experiments/proportion_time_spent.hpp"

int ProportionTimeSpent::addState()
{
    time_spent_in_each_states_.push_back(0.0);
    return time_spent_in_each_states_.size() - 1;
}

void ProportionTimeSpent::addTimeSpentInState(float time_spent, int index)
{
    time_spent_in_each_states_[index] += time_spent;
}

std::vector<float> ProportionTimeSpent::getProportionTimeSpent() const
{
    std::vector<float> proportion_time_spent;
    float total_time_spent = 0.0;
    for (float time_spent : time_spent_in_each_states_)
    {
        total_time_spent += time_spent;
    }
    for (float time_spent : time_spent_in_each_states_)
    {
        proportion_time_spent.push_back(time_spent / total_time_spent);
    }
    return proportion_time_spent;
}
    
//std::vector<float> time_spent_in_each_states_;
