#include "ros_queue/lib_queue/mean_stats.hpp"

MeanStats::MeanStats()
{
    time_0_ =std::chrono::high_resolution_clock::now();
}

void MeanStats::increaseArrivalMean(float new_arrival)
{
    arrival_sum_ += new_arrival;
    ++arrival_sample_size_;
    last_arrival_ = new_arrival;
};

double MeanStats::getArrivalTimeAverage()
{
    double current_time_point_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_0_).count();

    if (current_time_point_diff == 0)
    {
        return 0.0;
    }
    return arrival_sum_/current_time_point_diff*1000.0;
}

double MeanStats::getArrivalMean()
{
    return arrival_sum_/arrival_sample_size_;
}

double MeanStats::getArrivalTotal()
{
    return arrival_sum_;
}

double MeanStats::getLastArrival()
{
    return last_arrival_;
}

void MeanStats::increaseDepartureMean(float new_departure)
{
    departure_sum_ += new_departure;
    ++departure_sample_size_;
    last_departure_ = new_departure;
};

double MeanStats::getDepartureTimeAverage()
{
    double current_time_point_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_0_).count();

    if (current_time_point_diff == 0)
    {
        return 0.0;
    }
    return departure_sum_/current_time_point_diff*1000.0;
}

double MeanStats::getDepartureMean()
{
    return departure_sum_/departure_sample_size_;
}

double MeanStats::getDepartureTotal()
{
    return departure_sum_;
}

double MeanStats::getLastDeparture()
{
    return last_departure_;
}

void MeanStats::increaseRealDepartureMean(float new_departure)
{
    real_departure_sum_ += new_departure;
    ++real_departure_sample_size_;
    last_real_departure_ = new_departure;
}

double MeanStats::getRealDepartureTimeAverage()
{
    double current_time_point_diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_0_).count();

    if (current_time_point_diff == 0)
    {
        return 0.0;
    }
    return real_departure_sum_/current_time_point_diff*1000.0;
}

double MeanStats::getRealDepartureMean()
{
    return real_departure_sum_/real_departure_sample_size_;
}

double MeanStats::getRealDepartureTotal()
{
    return real_departure_sum_;
}

double MeanStats::getLastRealDeparture()
{
    return last_real_departure_;
}

double MeanStats::getChangeTimeAverage()
{
    return getArrivalTimeAverage() - getDepartureTimeAverage();
}

void MeanStats::increaseSizeMean(float new_queue_size)
{
    size_sum_ += new_queue_size;
    mean_sample_size += 1;
};

double MeanStats::getSizeMean()
{
    if(mean_sample_size == 0)
    {
        return 0.0;
    }
    return size_sum_/mean_sample_size;
}

void MeanStats::increaseConvertedRemainingMean(float current_remaining)
{
    converted_remaining_sum += current_remaining;
    remaining_sample_sum += 1;
};

double MeanStats::getConvertedRemainingMean()
{
    if(remaining_sample_sum == 0)
    {
        return 0.0;
    }
    return converted_remaining_sum/remaining_sample_sum;
}

double MeanStats::getSecondsSinceStart()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_0_).count()/1000.0;
}