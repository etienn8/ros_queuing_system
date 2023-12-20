#include "ros_queue_tests/distributions/inversed_cumulative_distribution.hpp"
#include <stdlib.h>
#include <time.h>

float InversedCumulativeDistribution::generateRandomSample()
{
    /* Initialize the random seed for the rand() function*/
    srand(time(NULL));

    float random_input_value = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float poisson_random_value = this->invertedCumulativeDistributionFunction(random_input_value);
    return poisson_random_value;
}