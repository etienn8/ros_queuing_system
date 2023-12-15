#pragma once

#include <random>

class InversedCumulativeDistribution
{
    public:
        /**
         * @brief Method that generates a random sample from a cumulative distribution.
         * @return Returns the generated sample.
        */
        float generateRandomSample();

        /**
         * @brief Mathematical inversion of the cumulative distribution function that gives a sample based on a given cumulative value.
         * @return Random variable value corresponding to the cumulative input value.
        */
        virtual float invertedCumulativeDistributionFunction(float random_input) = 0;
    
    protected:
        std::default_random_engine random_engine_;
};