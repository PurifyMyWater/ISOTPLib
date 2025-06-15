#include "CANMessageACKQueue.h"
#include <N_USData_Runner.h>

CANMessageACKQueue::CANMessageACKQueue(CANInterface& canInterface, OSInterface& osInterface, const char* tag)
{
    this->tag          = tag;
    mutex              = osInterface.osCreateMutex();
    this->canInterface = &canInterface;
}
CANMessageACKQueue::~CANMessageACKQueue()
{
    delete mutex;
}

void CANMessageACKQueue::saveAck(const CANInterface::ACKResult ack)
{
    if (!messageQueue.empty())
    {
        for (auto& [runner, runnerAck] : messageQueue)
        {
            if (runnerAck == CANInterface::ACK_NONE)
            {
                OSInterfaceLogDebug(this->tag, "Processing ACK %s for runner with N_AI=%s",
                                    CANInterface::ackResultToString(ack), nAiToString(runner->getN_AI()));
                runnerAck = ack; // Update the ACK result for the runner.
                break;
            }
        }
    }
    else
    {
        OSInterfaceLogWarning(this->tag, "No runners in queue to process ACK");
    }
}
void CANMessageACKQueue::runStep()
{
    if (const CANInterface::ACKResult ack = canInterface->getWriteFrameACK(); ack != CANInterface::ACK_NONE)
    {
        OSInterfaceLogDebug(this->tag, "ACK received: %s", CANInterface::ackResultToString(ack));
        if (mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
        {
            saveAck(ack);
            mutex->signal();
        }
        else
        {
            OSInterfaceLogError(this->tag, "Failed to acquire mutex for ACK storage of %s",
                                CANInterface::ackResultToString(ack));
        }
    }
}

void CANMessageACKQueue::runAvailableAckCallbacks()
{
    bool callbackHasRun = false;
    do
    {
        callbackHasRun = runNextAvailableAckCallback();
    }
    while (callbackHasRun);
}

bool CANMessageACKQueue::runNextAvailableAckCallback()
{
    bool callbackHasRun = false;
    if (mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        if (!messageQueue.empty())
        {
            if (const auto ack = messageQueue.front().second; ack != CANInterface::ACK_NONE)
            {
                auto* runner = messageQueue.front().first;

                messageQueue.pop_front();
                mutex->signal();

                OSInterfaceLogDebug(this->tag, "Running callback for runner with N_AI=%s and ACK=%s",
                                    nAiToString(runner->getN_AI()), CANInterface::ackResultToString(ack));
                runner->messageACKReceivedCallback(ack);
                callbackHasRun = true;
            }
            else
            {
                mutex->signal();
                callbackHasRun = false; // No more callbacks to run.
                OSInterfaceLogDebug(this->tag, "No ACK available for the first runner in the queue");
            }
        }
        else
        {
            mutex->signal();
            callbackHasRun = false; // No more callbacks to run.
            OSInterfaceLogDebug(this->tag, "No runners in queue to run callbacks");
        }
    }
    return callbackHasRun;
}

bool CANMessageACKQueue::writeFrame(N_USData_Runner& runner, CANFrame& frame)
{
    OSInterfaceLogDebug(this->tag, "Writing frame with N_AI=%s", nAiToString(frame.identifier));
    OSInterfaceLogVerbose(this->tag, "Writing frame: %s", frameToString(frame));
    bool res = canInterface->writeFrame(&frame);
    if (res && mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        messageQueue.emplace_back(&runner, CANInterface::ACK_NONE);
        mutex->signal();
    }
    return res;
}

bool CANMessageACKQueue::removeFromQueue(const N_AI runnerNAi)
{
    size_t res = 0;
    if (mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        res = messageQueue.remove_if([&runnerNAi](const auto& pair)
                                     { return pair.first->getN_AI().N_AI == runnerNAi.N_AI; });

        mutex->signal();
        OSInterfaceLogDebug(this->tag, "Runners with N_AI=%s not found in queue when attempting to remove it",
                            nAiToString(runnerNAi));
    }
    else
    {
        OSInterfaceLogError(this->tag, "Failed to acquire mutex for removing runner with N_AI=%s from queue",
                            nAiToString(runnerNAi));
    }
    return res > 0;
}
