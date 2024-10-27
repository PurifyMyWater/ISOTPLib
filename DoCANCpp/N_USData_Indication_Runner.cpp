#include "N_USData_Indication_Runner.h"

N_USData_Indication_Runner::N_USData_Indication_Runner(N_AI nAi, uint32_t* availableMemoryForRunners, uint8_t blockSize, STmin stMin, OSShim* osShim, CANShim* canShim): N_USData_Runner(nAi, osShim, canShim)
{
    this->runnerType = RunnerIndicationType;
}

N_USData_Indication_Runner::~N_USData_Indication_Runner()
{

}

N_Result N_USData_Indication_Runner::run_step(CANFrame* frame)
{
    return N_ERROR;
}
