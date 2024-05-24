#pragma once

#include <chrono>

struct MeanStats
{
    double arrival_sum_= 0.0f;
    long long arrival_sample_size_ = 0;

    double departure_sum_ = 0.0f;
    long long departure_sample_size_ = 0;
    
    double real_departure_sum_ = 0.0f;
    long long real_departure_sample_size_ = 0;

    double size_sum_ = 0.0f;
    long long mean_sample_size = 0; 
    double converted_remaining_sum = 0.0f;
    long long remaining_sample_sum = 0;

    bool should_compute_means_ = false;

    std::chrono::time_point<std::chrono::system_clock> time_0_;

    MeanStats();

    void increaseArrivalMean(float new_arrival);
    double getArrivalTimeAverage();
    double getArrivalMean();

    void increaseDepartureMean(float new_departure);
    double getDepartureTimeAverage();
    double getDepartureMean();

    void increaseRealDepartureMean(float new_departure);
    double getRealDepartureTimeAverage();
    double getRealDepartureMean();

    double getChangeTimeAverage();

    void increaseSizeMean(float new_queue_size);
    double getSizeMean();

    void increaseConvertedRemainingMean(float current_remaining);
    double getConvertedRemainingMean();

    double getSecondsSinceStart();
};