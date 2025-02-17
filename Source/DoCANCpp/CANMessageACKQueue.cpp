#include "CANMessageACKQueue.h"
#include <N_USData_Runner.h>


CANMessageACKQueue::CANMessageACKQueue(CANInterface& canShim) { this->canShim = &canShim; }

void CANMessageACKQueue::run_step()
{
    CANInterface::ACKResult ack = canShim->getWriteFrameACK();
    if (ack != CANInterface::ACK_NONE)
    {
        N_USData_Runner* runner = messageQueue.front();
        runner->messageACKReceivedCallback(ack);
        messageQueue.pop_front();
    }
}

bool CANMessageACKQueue::writeFrame(N_USData_Runner& runner, CANFrame& frame)
{
    bool res = canShim->writeFrame(&frame);
    if (res)
    {
        messageQueue.push_back(&runner);
    }
    return res;
}
