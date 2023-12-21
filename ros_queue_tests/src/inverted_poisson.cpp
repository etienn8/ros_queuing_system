#include "ros_queue_tests/distributions/inverted_poisson.hpp"

#include <cmath>
#include <random>

InvertedPoisson::InvertedPoisson(float lambda): lambda_(lambda)
{
    distribution_ = std::poisson_distribution<int>(lambda_);
}

float InvertedPoisson::invertedCumulativeDistributionFunction(float random_input)
{
    return static_cast<float>(distribution_(random_engine_));
}