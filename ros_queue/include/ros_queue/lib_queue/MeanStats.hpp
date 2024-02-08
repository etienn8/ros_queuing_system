#pragma once

#include <chrono>

struct MeanStats
{
    double arrival_sum_= 0.0f;
    double departure_sum_ = 0.0f;
    double size_sum_ = 0.0f;
    long long mean_sample_size = 0; 

    bool should_compute_means_ = false;

    std::chrono::time_point<std::chrono::system_clock> time_0_;

    MeanStats()
    {
        time_0_ =std::chrono::high_resolution_clock::now();
    }

    void increaseArrivalMean(float new_arrival)
    {
        arrival_sum_ += new_arrival;
    };

    double getArrivalMean()
    {
        double current_time_point_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_0_).count();

        return arrival_sum_/current_time_point_diff;
    }

    void increaseDepartureMean(float new_departure)
    {
        departure_sum_ += new_departure;
    };

    double getDepartureMean()
    {
        double current_time_point_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_0_).count();

        return departure_sum_/current_time_point_diff;
    }

    void increaseSizeMean(float new_queue_size)
    {
        size_sum_ += new_queue_size;
        mean_sample_size += 1;
    };

    double getSizeMean()
    {
        return size_sum_/mean_sample_size;
    }
};