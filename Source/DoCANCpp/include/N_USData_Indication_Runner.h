#ifndef N_USDATA_INDICATION_RUNNER_H
#define N_USDATA_INDICATION_RUNNER_H

#include "N_USData_Runner.h"

#include "Atomic_int64_t.h"

#include <Timer_N.h>

#warning TODO remove this include after removing tag before merging this branch
#include <cstring> // TODO remove this include after removing tag before merging this branch

// Class that handles the indication aka reception of a message
class N_USData_Indication_Runner : public N_USData_Runner
{
public:
    N_USData_Indication_Runner(N_AI nAi, Atomic_int64_t& availableMemoryForRunners, uint8_t blockSize, STmin stMin,
                               OSInterface& osInterface, CANMessageACKQueue& canMessageACKQueue);

    ~N_USData_Indication_Runner() override;

    N_Result run_step(CANFrame* receivedFrame) override;

    [[nodiscard]] bool awaitingMessage() const override;

    [[nodiscard]] uint32_t getNextRunTime() const override;

    void messageACKReceivedCallback(CANInterface::ACKResult success) override;

    bool setBlockSize(uint8_t blockSize);

    bool setSTmin(STmin stMin);

    [[nodiscard]] N_AI getN_AI() const override;

    [[nodiscard]] uint8_t* getMessageData() const override;

    [[nodiscard]] uint32_t getMessageLength() const override;

    [[nodiscard]] N_Result getResult() const override;

    [[nodiscard]] Mtype getMtype() const override;

    [[nodiscard]] RunnerType getRunnerType() const override;

private:
    N_Result run_step_notRunning(const CANFrame* receivedFrame);
    N_Result run_step_CF(const CANFrame* receivedFrame);

    N_Result               sendFCFrame(FlowStatus fs);
    [[nodiscard]] uint32_t getNextTimeoutTime() const;
    N_Result               checkTimeouts();

    using InternalStatus_t = enum { NOT_RUNNING, AWAITING_FC_ACK, AWAITING_CF, ERROR };

    OSInterface_Mutex* mutex;

    N_AI     nAi;
    Mtype    mType;
    uint8_t* messageData;
    int64_t  messageLength;
    uint8_t  blockSize;
    uint8_t  effectiveBlockSize;
    STmin    stMin{};
    STmin    effectiveStMin{};

    char* tag = strdup("N_USData_Request_Runner");

    N_Result         result;
    uint32_t         lastRunTime;
    uint8_t          sequenceNumber;
    InternalStatus_t internalStatus;
    Atomic_int64_t*  availableMemoryForRunners;
    uint32_t         messageOffset;
    int16_t          cfReceivedInThisBlock;

    Timer_N* timerN_Ar; // Timer for sending a frame
    Timer_N* timerN_Br; // Timer that holds the time since the last FF or CF to the next FC.
    Timer_N* timerN_Cr; // Timer that holds the time since the last FC to the next FC.

    OSInterface*        osInterface;
    CANMessageACKQueue* CanMessageACKQueue{};
};

#endif // N_USDATA_INDICATION_RUNNER_H
