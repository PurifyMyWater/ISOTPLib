#ifndef N_USDATA_REQUEST_RUNNER_H
#define N_USDATA_REQUEST_RUNNER_H

#include "N_USData_Runner.h"
#include "Atomic_int64_t.h"

// Class that handles the request aka transmission of a message
class N_USData_Request_Runner : public N_USData_Runner
{
public:
    N_USData_Request_Runner(bool* result, N_AI nAi, Atomic_int64_t& availableMemoryForRunners, Mtype mType, const uint8_t* messageData, uint32_t messageLength, OSShim& osShim, CANShim& canShim);

    ~N_USData_Request_Runner() override;

    N_Result run_step(CANFrame* receivedFrame) override;

    [[nodiscard]] bool awaitingMessage() const override;

    [[nodiscard]] uint32_t getNextRunTime() const override;
private:
    Atomic_int64_t* availableMemoryForRunners; // TODO USE 64 bit int
};

#endif //N_USDATA_REQUEST_RUNNER_H
