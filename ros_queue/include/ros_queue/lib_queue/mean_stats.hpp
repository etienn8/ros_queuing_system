#pragma once

#include <chrono>

struct MeanStats
{
    double arrival_sum_= 0.0f;
    double departure_sum_ = 0.0f;
    double real_departure_sum_ = 0.0f;
    double size_sum_ = 0.0f;
    long long mean_sample_size = 0; 
    double converted_remaining_sum = 0.0f;
    long long remaining_sample_sum = 0;

    bool should_compute_means_ = false;

    std::chrono::time_point<std::chrono::system_clock> time_0_;

    MeanStats();

    void increaseArrivalMean(float new_arrival);

    double getArrivalTimeAverage();

    void increaseDepartureMean(float new_departure);

    double getDepartureTimeAverage();

    void increaseRealDepartureMean(float new_departure);

    double getRealDepartureTimeAverage();

    double getChangeTimeAverage();

    void increaseSizeMean(float new_queue_size);

    double getSizeMean();

    void increaseConvertedRemainingMean(float current_remaining);

    double getConvertedRemainingMean();

    double getSecondsSinceStart();
};