#pragma once

#include "inversed_cumulative_distribution.hpp"

class InvertedBernoulli: public InversedCumulativeDistribution
{
    public:
        InvertedBernoulli(float high_value, float low_value, float probability_of_high_value);

        float invertedCumulativeDistributionFunction(float random_input) override;

    private:
        float high_value_=0.0f;
        float low_value_=0.0f;
        float probability_of_high_value_=0.0f;;
};