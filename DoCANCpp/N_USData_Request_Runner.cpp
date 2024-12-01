#include "N_USData_Request_Runner.h"
#include "Atomic_int64_t.h"

#include <cstring>


N_USData_Request_Runner::N_USData_Request_Runner(bool* result, N_AI nAi, Atomic_int64_t& availableMemoryForRunners, Mtype mType, const uint8_t* messageData, uint32_t messageLength, OSShim& osShim,
                                                 CANShim& canShim) : N_USData_Runner(nAi, osShim, canShim)
{
    this->internalStatus = INVALID;
    this->messageOffset = 0;
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

        if (messageLength <= MAX_SF_MESSAGE_LENGTH)
        {
            this->internalStatus = NOT_RUNNING_SF;
        }
        else
        {
            this->internalStatus = NOT_RUNNING_FF;
        }

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
    switch (internalStatus)
    {
        case NOT_RUNNING_SF:
            return run_step_SF(receivedFrame);
        case NOT_RUNNING_FF:
            return run_step_FF(receivedFrame);
        case AWAITING_FC: // We got the message or timeout.
        {

        }
        default:
            result = N_ERROR;
            return N_ERROR;
    }
}

N_Result N_USData_Request_Runner::run_step_FF(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        result = N_ERROR;
        return result;
    }
    else
    {
        CANFrame ffFrame = NewCANFrameDoCANCpp();
        ffFrame.identifier = nAi;

        if (messageLength < MIN_FF_DL_WITH_ESCAPE_SEQUENCE)
        {
            ffFrame.data[0] = (FF_CODE << 4) | (messageLength >> 8); // N_PCI_FF (0b0001xxxx) | messageLength (0bxxxxllll)
            ffFrame.data[1] = messageLength & 0b11111111; // messageLength LSB

            memcpy(&ffFrame.data[2], messageData, 6); // Payload data
            messageOffset = 6;
        }
        else
        {
            ffFrame.data[0] = FF_CODE << 4; // N_PCI_FF (0b00010000)
            ffFrame.data[1] = 0;
            *reinterpret_cast<uint32_t*>(&ffFrame.data[2]) = static_cast<uint32_t>(messageLength); // copy messageLength (4 bytes) in the bytes #2 to #5.

            memcpy(&ffFrame.data[6], messageData, 2); // Payload data
            messageOffset = 2;
        }

        ffFrame.data_length_code = 8;

        if (canShim->writeFrame(&ffFrame))
        {
            internalStatus = AWAITING_FC;
            result = IN_PROGRESS;
            return result;
        }

        internalStatus = INVALID;
        result = N_ERROR;
        return result;
    }
}

N_Result N_USData_Request_Runner::run_step_SF(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        result = N_ERROR;
        return result;
    }

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


bool N_USData_Request_Runner::awaitingMessage() const
{
    return internalStatus == AWAITING_FC;
}

uint32_t N_USData_Request_Runner::getNextRunTime() const
{
    return 0; // TODO NOT IMPLEMENTED
}
