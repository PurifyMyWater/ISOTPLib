#ifndef N_USDATA_REQUEST_RUNNER_H
#define N_USDATA_REQUEST_RUNNER_H

#include "Atomic_int64_t.h"
#include "N_USData_Runner.h"

// Class that handles the request aka transmission of a message
class N_USData_Request_Runner : public N_USData_Runner
{
public:
    N_USData_Request_Runner(bool* result, N_AI nAi, Atomic_int64_t& availableMemoryForRunners, Mtype mType, const uint8_t* messageData, uint32_t messageLength, OSShim& osShim,
                            CANMessageACKQueue& canMessageACKQueue);

    ~N_USData_Request_Runner() override;

    N_Result run_step(CANFrame* receivedFrame) override;

    [[nodiscard]] bool awaitingMessage() const override;

    [[nodiscard]] uint32_t getNextRunTime() const override;

    void messageACKReceivedCallback(CANShim::ACKResult success) override;

private:
    N_Result run_step_SF(const CANFrame* receivedFrame);
    N_Result run_step_FF(const CANFrame* receivedFrame);

    using InternalStatus_t = enum { NOT_RUNNING_SF, NOT_RUNNING_FF, AWAITING_FC, INVALID };

    Atomic_int64_t* availableMemoryForRunners;
    uint32_t messageOffset;
    InternalStatus_t internalStatus;
};

#endif // N_USDATA_REQUEST_RUNNER_H
