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

        if (current_time_point_diff == 0)
        {
            return 0.0;
        }
        return arrival_sum_/current_time_point_diff*1000.0;
    }

    void increaseDepartureMean(float new_departure)
    {
        departure_sum_ += new_departure;
    };

    double getDepartureMean()
    {
        double current_time_point_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_0_).count();

        if (current_time_point_diff == 0)
        {
            return 0.0;
        }
        return departure_sum_/current_time_point_diff*1000.0;
    }
   void increaseRealDepartureMean(float new_departure)
    {
        real_departure_sum_ += new_departure;
    };

    double getRealDepartureMean()
    {
        double current_time_point_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_0_).count();

        if (current_time_point_diff == 0)
        {
            return 0.0;
        }
        return real_departure_sum_/current_time_point_diff*1000.0;
    }

    double getChangeMean()
    {
        return getArrivalMean() - getDepartureMean();
    }

    void increaseSizeMean(float new_queue_size)
    {
        size_sum_ += new_queue_size;
        mean_sample_size += 1;
    };

    double getSizeMean()
    {
        if(mean_sample_size == 0)
        {
            return 0.0;
        }
        return size_sum_/mean_sample_size;
    }

    void increaseConvertedRemainingMean(float current_remaining)
    {
        converted_remaining_sum += current_remaining;
        remaining_sample_sum += 1;
    };

    double getConvertedRemainingMean()
    {
        if(remaining_sample_sum == 0)
        {
            return 0.0;
        }
        return converted_remaining_sum/remaining_sample_sum;
    }

    double getSecondsSinceStart()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_0_).count()/1000.0;
    }
};