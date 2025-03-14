#include "CANMessageACKQueue.h"
#include <N_USData_Runner.h>


CANMessageACKQueue::CANMessageACKQueue(CANInterface& canInterface) { this->canInterface = &canInterface; }

void CANMessageACKQueue::run_step()
{
    CANInterface::ACKResult ack = canInterface->getWriteFrameACK();
    if (ack != CANInterface::ACK_NONE)
    {
        OSInterfaceLogDebug(TAG, "ACK received: %s", CANInterface::ackResultToString(ack));
        N_USData_Runner* runner = messageQueue.front();
        runner->messageACKReceivedCallback(ack);
        messageQueue.pop_front();
    }
}

bool CANMessageACKQueue::writeFrame(N_USData_Runner& runner, CANFrame& frame)
{
    OSInterfaceLogDebug(TAG, "Writing frame with N_AI=%s: ", nAiToString(frame.identifier));
    OSInterfaceLogVerbose(TAG, "Writing frame: %s", frameToString(frame));
    bool res = canInterface->writeFrame(&frame);
    if (res)
    {
        messageQueue.push_back(&runner);
    }
    return res;
}
