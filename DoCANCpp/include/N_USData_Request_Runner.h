#ifndef N_USDATA_REQUEST_RUNNER_H
#define N_USDATA_REQUEST_RUNNER_H

#include "Atomic_int64_t.h"
#include "N_USData_Runner.h"
#include "Timer_N.h"

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
    N_Result run_step_CF(const CANFrame* receivedFrame);
    N_Result run_step_FC(const CANFrame* receivedFrame, bool firstFC = false);

    N_Result parseFCFrame(const CANFrame* receivedFrame, FlowStatus& fs, uint8_t& blockSize, STmin& stMin);
    [[nodiscard]] uint32_t getNextTimeoutTime() const;
    N_Result checkTimeouts() override;
    N_Result sendCFFrame();

    using InternalStatus_t = enum { NOT_RUNNING_SF, AWAITING_SF_ACK, NOT_RUNNING_FF, AWAITING_FF_ACK, AWAITING_FirstFC, AWAITING_FC, SEND_CF, AWAITING_CF_ACK, MESSAGE_SENT, ERROR};

    Atomic_int64_t* availableMemoryForRunners;
    uint32_t messageOffset;
    InternalStatus_t internalStatus;
    int16_t cfSentInThisBlock;

    Timer_N* timerN_As; // Timer for sending a frame
    Timer_N* timerN_Bs; // Timer that holds the time since the last FF or CF to the next CF.
    Timer_N* timerN_Cs; // Timer that calls out once STmin has passed.
};

#endif // N_USDATA_REQUEST_RUNNER_H
