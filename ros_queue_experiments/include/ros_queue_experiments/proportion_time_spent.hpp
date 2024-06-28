#pragma once

#include <vector>

class ProportionTimeSpent
{
    public:
        /**
         * @brief Add a new state to the proportion time spent.
         * @return The index of the new state.
        */
        int addState();

        /**
         * @brief Add time spent in the given state.
         * @param time_spent Time to add in the time spent for the given index.
         * @param index Represents the index of the state.
        */
        void addTimeSpentInState(float time_spent, int index);

        /**
         * @brief Get the proportion of time spent in each state.
         * @return The proportion of time spent in each state between 0.0 to 1.0
        */
        std::vector<float> getProportionTimeSpent() const;
    
    private:
        std::vector<float> time_spent_in_each_states_;
};