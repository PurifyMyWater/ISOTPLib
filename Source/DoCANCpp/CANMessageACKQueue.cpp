#include "CANMessageACKQueue.h"
#include <N_USData_Runner.h>


CANMessageACKQueue::CANMessageACKQueue(CANInterface& canInterface, OSInterface& osInterface)
{
    mutex = osInterface.osCreateMutex();
    this->canInterface = &canInterface;
}
CANMessageACKQueue::~CANMessageACKQueue() { delete mutex; }

void CANMessageACKQueue::run_step()
{
    if (CANInterface::ACKResult ack = canInterface->getWriteFrameACK(); ack != CANInterface::ACK_NONE)
    {
        OSInterfaceLogDebug(TAG, "ACK received: %s", CANInterface::ackResultToString(ack));
        
        if (mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
        {
            N_USData_Runner* runner = messageQueue.front();
            messageQueue.pop_front();
            mutex->signal();
            runner->messageACKReceivedCallback(ack);
        }
    }
}

bool CANMessageACKQueue::writeFrame(N_USData_Runner& runner, CANFrame& frame)
{
    OSInterfaceLogDebug(TAG, "Writing frame with N_AI=%s: ", nAiToString(frame.identifier));
    OSInterfaceLogVerbose(TAG, "Writing frame: %s", frameToString(frame));
    bool res = canInterface->writeFrame(&frame);
    if (res && mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        messageQueue.push_back(&runner);
        mutex->signal();
    }
    return res;
}
