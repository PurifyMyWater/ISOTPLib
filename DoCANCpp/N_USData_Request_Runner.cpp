#include "N_USData_Request_Runner.h"
#include "Atomic_int64_t.h"

#include <cstring>


N_USData_Request_Runner::N_USData_Request_Runner(bool* result, N_AI nAi, Atomic_int64_t& availableMemoryForRunners, Mtype mType, const uint8_t* messageData, uint32_t messageLength, OSShim& osShim, CANShim& canShim) : N_USData_Runner(nAi, osShim, canShim)
{
    this->runnerType = RunnerRequestType;
    int64_t availableMemory;
    this->messageData = nullptr;
    if (availableMemoryForRunners.get(&availableMemory) && availableMemory > messageLength && messageData != nullptr)
    {
        this->messageData = reinterpret_cast<uint8_t*>(osShim.osMalloc(messageLength * sizeof(uint8_t)));
        memcpy(this->messageData, messageData, messageLength);
        availableMemoryForRunners.sub(messageLength);
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
    if(this->messageData == nullptr)
    {
        return;
    }

    osShim->osFree(this->messageData);
    availableMemoryForRunners->add(messageLength);
}

N_Result N_USData_Request_Runner::run_step(CANFrame* frame)
{
    return N_ERROR;
}
