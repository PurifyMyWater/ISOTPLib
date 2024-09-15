//
// Created by victor on 9/08/24.
//

#ifndef CANMASTER_OSSHIM_MUTEX_H
#define CANMASTER_OSSHIM_MUTEX_H

#include <cstdint>

class OSShim_Mutex
{
public:
    virtual void signal() = 0;
    virtual bool wait(uint32_t max_time_to_wait_ms) = 0;
};

#endif //CANMASTER_OSSHIM_MUTEX_H
