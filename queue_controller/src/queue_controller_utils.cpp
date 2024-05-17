#include "queue_controller/queue_controller_utils.hpp"

#include <chrono>

using std::string;

namespace queue_controller_utils
{
    std::chrono::time_point<std::chrono::system_clock> updateTimePointAndGetTimeDifferenceMS(std::chrono::time_point<std::chrono::system_clock> time_0, double& time_diff)
    {
        std::chrono::time_point<std::chrono::system_clock> new_time_point = std::chrono::high_resolution_clock::now();
        time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(new_time_point - time_0).count();
        return new_time_point;
    }

    bool isRenewalController(const string& controller_type)
    {
        return controller_type == "renewal_min_drift_plus_penalty";
    }
}