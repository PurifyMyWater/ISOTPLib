#include "CANShim.h"
#include <cassert>


CANShim::CANShim(CANShim::FrameAvailableCallback frameAvailableCallback, CANShim::ReadFrameCallback readFrameCallback, CANShim::WriteFrameCallback writeFrameCallback, ActiveCallback activeCallback)
{
    if(frameAvailableCallback == nullptr)
    {
        assert(false && "CANShim(): frameAvailableCallback is NULL");
    }
    if(readFrameCallback == nullptr)
    {
        assert(false && "CANShim(): readFrameCallback is NULL");
    }
    if(writeFrameCallback == nullptr)
    {
        assert(false && "CANShim(): writeFrameCallback is NULL");
    }
    if(activeCallback == nullptr)
    {
        assert(false && "CANShim(): activeCallback is NULL");
    }

    this->frameAvailableCallback = frameAvailableCallback;
    this->readFrameCallback = readFrameCallback;
    this->writeFrameCallback = writeFrameCallback;
    this->activeCallback = activeCallback;
}

uint32_t CANShim::frameAvailable()
{
    return this->frameAvailableCallback();
}

bool CANShim::readFrame(CANShim::CANFrame* frame)
{
    return this->readFrameCallback(frame);
}

bool CANShim::writeFrame(CANShim::CANFrame* frame)
{
    return this->writeFrameCallback(frame);
}

bool CANShim::active()
{
    return this->activeCallback();
}
