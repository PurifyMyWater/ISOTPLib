#include "N_USData_Request_Runner.h"
#include "Atomic_int64_t.h"

#include <cstring>


N_USData_Request_Runner::N_USData_Request_Runner(bool* result, N_AI nAi, Atomic_int64_t& availableMemoryForRunners, Mtype mType, const uint8_t* messageData, uint32_t messageLength, OSShim& osShim,
                                                 CANShim& canShim) : N_USData_Runner(nAi, osShim, canShim)
{
    this->runnerType = RunnerRequestType;
    this->messageData = nullptr;
    this->messageLength = messageLength;
    if (availableMemoryForRunners.subIfResIsGreaterThanZero(this->messageLength) && messageData != nullptr)
    {
        this->messageData = static_cast<uint8_t*>(osShim.osMalloc(this->messageLength * sizeof(uint8_t)));
        memcpy(this->messageData, messageData, this->messageLength);
        this->mType = mType;
        this->nAi = nAi;
        this->availableMemoryForRunners = &availableMemoryForRunners;
        *result = true;
    }
    else
    {
        *result = false;
    }
}

N_USData_Request_Runner::~N_USData_Request_Runner()
{
    if (this->messageData == nullptr)
    {
        return;
    }

    osShim->osFree(this->messageData);
    availableMemoryForRunners->add(this->messageLength);
}

N_Result N_USData_Request_Runner::run_step(CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        result = N_ERROR;
        return result;
    }
    else
    {
        if (messageLength <= MAX_SF_MESSAGE_LENGTH)
        {
            CANFrame sfFrame = NewCANFrameDoCANCpp();
            sfFrame.identifier = nAi;

            sfFrame.data[0] = messageLength; // N_PCI_SF (0b0000xxxx) | messageLength (0bxxxxllll)
            memcpy(&sfFrame.data[1], messageData, messageLength); // Payload data

            sfFrame.data_length_code = messageLength + 1; // 1 byte for N_PCI_SF

            if (canShim->writeFrame(&sfFrame))
            {
                result = N_OK;
                return result;
            }
            result = N_ERROR;
            return result;
        }
        else
        {
            result = N_ERROR;
            return result; // Unexpected data length
        }
    }
}

bool N_USData_Request_Runner::awaitingMessage() const
{
    return false; // TODO NOT IMPLEMENTED
}

uint32_t N_USData_Request_Runner::getNextRunTime() const
{
    return 0; // TODO NOT IMPLEMENTED
}
