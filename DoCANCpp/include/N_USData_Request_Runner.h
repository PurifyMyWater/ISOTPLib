#ifndef N_USDATA_REQUEST_RUNNER_H
#define N_USDATA_REQUEST_RUNNER_H

#include "N_USData_Runner.h"

// Class that handles the request aka transmission of a message
class N_USData_Request_Runner : public N_USData_Runner
{
public:
    N_USData_Request_Runner(bool* result, N_AI nAi, Atomic_uint32_t& availableMemoryForRunners, Mtype mType, const uint8_t* messageData, uint32_t messageLength, OSShim& osShim, CANShim& canShim);

    ~N_USData_Request_Runner() override;

    N_Result run_step(CANFrame* frame) override;
private:
    Atomic_uint32_t* availableMemoryForRunners;
};

#endif //N_USDATA_REQUEST_RUNNER_H
