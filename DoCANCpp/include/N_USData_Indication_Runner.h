#ifndef CANMASTER_N_USDATA_INDICATION_RUNNER_H
#define CANMASTER_N_USDATA_INDICATION_RUNNER_H

#include "N_USData_Runner.h"

// Class that handles the indication aka reception of a message
class N_USData_Indication_Runner : public N_USData_Runner
{
public:
    N_USData_Indication_Runner(N_AI nAi, uint32_t* availableMemoryForRunners, OSShim* osShim, CANShim* canShim);

    ~N_USData_Indication_Runner() override;

    N_Result run_step(CANShim::CANFrame* frame) override;
};

#endif //CANMASTER_N_USDATA_INDICATION_RUNNER_H
