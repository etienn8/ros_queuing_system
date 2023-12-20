#include "ros_queue_tests/distributions/inverted_bernoulli.hpp"

InvertedBernoulli::InvertedBernoulli(float high_value, float low_value, 
                                     float probability_of_high_value): 
                                     high_value_(high_value), low_value_(low_value),
                                     probability_of_high_value_(probability_of_high_value) {};


float InvertedBernoulli::invertedCumulativeDistributionFunction(float random_input)
{
    if (random_input > probability_of_high_value_)
    {
        return high_value_;
    }

    return low_value_;
}