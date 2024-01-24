#include "queue_server/queue_server_utils.hpp"

namespace queue_server_utils
{
    bool isQueueTypeVirtual(const string queue_type)
    {
        if(queue_type == "inequality_constraint_virtual_queue" || 
           queue_type == "equality_constraint_virtual_queue") 
        {
            return true;
        }
        return false;
    }
}