#include "N_USData_Indication_Runner.h"

#include <cstring>

N_USData_Indication_Runner::N_USData_Indication_Runner(N_AI nAi, Atomic_int64_t& availableMemoryForRunners, uint8_t blockSize, STmin stMin, OSShim& osShim, CANShim& canShim) :
    N_USData_Runner(nAi, osShim, canShim)
{
    this->runnerType = RunnerIndicationType;
    this->nAi = nAi;
    this->blockSize = blockSize;
    this->stMin = stMin;
    this->availableMemoryForRunners = &availableMemoryForRunners;
    this->osShim = &osShim;
    this->canShim = &canShim;
    this->messageData = nullptr;
}

N_USData_Indication_Runner::~N_USData_Indication_Runner()
{
    if (this->messageData != nullptr)
    {
        this->osShim->osFree(messageData);
        this->availableMemoryForRunners->add(messageLength);
    }
}

N_Result N_USData_Indication_Runner::run_step(CANFrame* frame)
{
    if (frame != nullptr) // TODO take into account the different types of addressing.
    {
        this->mType = Mtype_Diagnostics; // DoCANCpp already checks if the frame is a diagnostics frame by looking at the N_TAType
        if ((frame->data[0] >> 4) == SF_CODE) // messageLength < MAX_SF_MESSAGE_LENGTH
        {
            messageLength = frame->data[0] & 0x0F;
            messageData = static_cast<uint8_t*>(osShim->osMalloc(messageLength));
            memcpy(messageData, &frame->data[1], messageLength);
            result = N_OK;
            return result;
        }
        else
        {
            result = N_ERROR;
            return result; // INVALID SF
        }
    }
    else
    {
        result = N_ERROR;
        return result; // NO FRAME
    }
}

bool N_USData_Indication_Runner::awaitingMessage() const
{
    return true; // TODO NOT IMPLEMENTED
}

uint32_t N_USData_Indication_Runner::getNextRunTime() const
{
    return 0; // TODO NOT IMPLEMENTED
}
