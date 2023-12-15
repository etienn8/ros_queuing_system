#include "ros_queue_tests/inverted_poisson.hpp"

#include <cmath> 

float InvertedPoisson::invertedCumulativeDistributionFunction(float random_input)
{
    // https://web.mit.edu/urban_or_book/www/book/chapter7/7.1.3.html
    return -static_cast<float>(log(random_input)/lambda_);
}