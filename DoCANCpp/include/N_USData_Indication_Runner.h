#ifndef N_USDATA_INDICATION_RUNNER_H
#define N_USDATA_INDICATION_RUNNER_H

#include "N_USData_Runner.h"

#include "Atomic_int64_t.h"

#include <Timer_N.h>

// Class that handles the indication aka reception of a message
class N_USData_Indication_Runner : public N_USData_Runner
{
public:
    N_USData_Indication_Runner(N_AI nAi, Atomic_int64_t& availableMemoryForRunners, uint8_t blockSize, STmin stMin, OSShim& osShim, CANMessageACKQueue& canMessageACKQueue);

    ~N_USData_Indication_Runner() override;

    N_Result run_step(CANFrame* receivedFrame) override;

    [[nodiscard]] bool awaitingMessage() const override;

    [[nodiscard]] uint32_t getNextRunTime() const override;

    void messageACKReceivedCallback(CANShim::ACKResult success) override;

private:
    N_Result run_step_notRunning(CANFrame* receivedFrame);
    N_Result run_step_CF(CANFrame* receivedFrame);

    N_Result sendFCFrame(FlowStatus fs);
    uint32_t getNextTimeoutTime() const;
    N_Result checkTimeouts() override;

    using InternalStatus_t = enum { NOT_RUNNING, AWAITING_FC_ACK, AWAITING_CF, ERROR };

    InternalStatus_t internalStatus;
    Atomic_int64_t* availableMemoryForRunners;

    uint32_t messageOffset;
    int16_t cfReceivedInThisBlock;

    Timer_N* timerN_Ar; // Timer for sending a frame
    Timer_N* timerN_Br; // Timer that holds the time since the last FF or CF to the next FC.
    Timer_N* timerN_Cr; // Timer that holds the time since the last FC to the next FC.
};

#endif // N_USDATA_INDICATION_RUNNER_H
