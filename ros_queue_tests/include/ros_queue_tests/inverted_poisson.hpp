#pragma once

#include "inversed_cumulative_distribution.hpp"

class InvertedPoisson: public InversedCumulativeDistribution
{
    public:
        InvertedPoisson(float lambda):lambda_(lambda) {};

        float invertedCumulativeDistributionFunction(float random_input) override;

    private:
        float lambda_=0.0f;
};