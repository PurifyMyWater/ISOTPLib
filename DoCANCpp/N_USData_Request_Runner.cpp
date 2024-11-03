#include "N_USData_Request_Runner.h"


N_USData_Request_Runner::N_USData_Request_Runner(N_AI nAi, Mtype mType, uint8_t* messageData, uint32_t messageLength, OSShim* osShim, CANShim* canShim) : N_USData_Runner(nAi, osShim, canShim)
{
    this->runnerType = RunnerRequestType;
}

N_USData_Request_Runner::~N_USData_Request_Runner()
= default;

N_Result N_USData_Request_Runner::run_step(CANFrame* frame)
{
    return N_ERROR;
}
