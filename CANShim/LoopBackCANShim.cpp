#include "LoopBackCANShim.h"

#include <deque>

static std::deque<CANShim::CANFrame> loopBackQueue;

uint32_t loopBack_shim_can_frameAvailable()
{
    return loopBackQueue.size();
}

bool loopBack_shim_can_readFrame(CANShim::CANFrame* frame)
{
    if(loopBackQueue.empty())
    {
        return false;
    }

    *frame = loopBackQueue.front();
    loopBackQueue.pop_front();
    return true;
}

bool loopBack_shim_can_writeFrame(CANShim::CANFrame* frame)
{
    loopBackQueue.push_back(*frame);
    return true;
}

bool loopBack_shim_can_active()
{
    return true;
}

CANShim loopBackCanShim(loopBack_shim_can_frameAvailable, loopBack_shim_can_readFrame, loopBack_shim_can_writeFrame, loopBack_shim_can_active);
