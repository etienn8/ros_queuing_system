#pragma once

#include <string>

using std::string;

namespace queue_server_utils
{
    /**
     * @brief Returns if a queue type is a known virtual queue type.
     * @param queue_type Name of the queue type to check.
     * @return Returns true if the queue type is a known virtual type and false otherwise 
     * (including types that don't exist).
    */
    bool isQueueTypeVirtual(const string queue_type);
}