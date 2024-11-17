#ifndef N_USDATA_INDICATION_RUNNER_H
#define N_USDATA_INDICATION_RUNNER_H

#include "N_USData_Runner.h"
#include "DoCANCpp.h"

// Class that handles the indication aka reception of a message
class N_USData_Indication_Runner : public N_USData_Runner
{
public:
    N_USData_Indication_Runner(N_AI nAi, Atomic_uint32_t& availableMemoryForRunners, uint8_t blockSize, STmin stMin, OSShim& osShim, CANShim& canShim);

    ~N_USData_Indication_Runner() override;

    N_Result run_step(CANFrame* frame) override;
private:
    Atomic_uint32_t* availableMemoryForRunners;
};

#endif //N_USDATA_INDICATION_RUNNER_H
