#include "N_USData_Indication_Runner.h"

#include <cstring>

N_USData_Indication_Runner::N_USData_Indication_Runner(N_AI nAi, Atomic_int64_t& availableMemoryForRunners, uint8_t blockSize, STmin stMin, OSShim& osShim, CANShim& canShim): N_USData_Runner(nAi, osShim, canShim)
{
    this->runnerType = RunnerIndicationType;
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
}

bool N_USData_Indication_Runner::awaitingMessage() const
{
    return true; // TODO NOT IMPLEMENTED
}

uint32_t N_USData_Indication_Runner::getNextRunTime() const
{
    return 0; // TODO NOT IMPLEMENTED
}
