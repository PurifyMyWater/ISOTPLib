#ifndef N_USDATA_INDICATION_RUNNER_H
#define N_USDATA_INDICATION_RUNNER_H

#include "Atomic_int64_t.h"
#include "CANMessageACKQueue.h"
#include "N_USData_Runner.h"
#include "Timer_N.h"

constexpr char    N_USDATA_INDICATION_RUNNER_STATIC_TAG[] = "ISOTP_IndicationRunner_";
constexpr int32_t N_USDATA_INDICATION_RUNNER_TAG_SIZE =
    MAX_N_AI_STR_SIZE + sizeof(N_USDATA_INDICATION_RUNNER_STATIC_TAG);

// Class that handles the indication aka reception of a message
class N_USData_Indication_Runner : public N_USData_Runner
{
public:
    N_USData_Indication_Runner(bool& result, N_AI nAi, Atomic_int64_t& availableMemoryForRunners, uint8_t blockSize,
                               STmin stMin, OSInterface& osInterface, CANMessageACKQueue& canMessageACKQueue);

    ~N_USData_Indication_Runner() override;

    N_Result runStep(CANFrame* receivedFrame) override;

    [[nodiscard]] uint32_t getNextRunTime() override;

    void messageACKReceivedCallback(CANInterface::ACKResult success) override;

    bool setBlockSize(uint8_t blockSize);

    bool setSTmin(STmin stMin);

    [[nodiscard]] N_AI getN_AI() const override;

    [[nodiscard]] uint8_t* getMessageData() const override;

    [[nodiscard]] uint32_t getMessageLength() const override;

    [[nodiscard]] N_Result getResult() const override;

    [[nodiscard]] Mtype getMtype() const override;

    [[nodiscard]] RunnerType getRunnerType() const override;

    [[nodiscard]] const char* getTAG() const override;

    [[nodiscard]] bool isThisFrameForMe(const CANFrame& frame) const override;

private:
    N_Result runStep_internal(const CANFrame* receivedFrame);
    N_Result runStep_notRunning(const CANFrame* receivedFrame);
    N_Result runStep_holdFrame(const CANFrame* receivedFrame);
    N_Result runStep_CF(const CANFrame* receivedFrame);
    N_Result runStep_FC_CTS(const CANFrame* receivedFrame);

    void FC_ACKReceivedCallback(CANInterface::ACKResult success);

    N_Result               sendFCFrame(FlowStatus fs);
    [[nodiscard]] uint32_t getNextTimeoutTime() const;
    N_Result               checkTimeouts();
    [[nodiscard]] bool     awaitingFrame(const CANFrame& frame) const;

    using InternalStatus_t = enum { NOT_RUNNING, SEND_FC, AWAITING_FC_ACK, AWAITING_CF, MESSAGE_RECEIVED, ERROR };

    [[nodiscard]] static const char* internalStatusToString(InternalStatus_t status);

    N_AI     nAi;
    Mtype    mType;
    uint8_t* messageData{};
    int64_t  messageLength;
    uint8_t  blockSize;
    uint8_t  effectiveBlockSize;
    STmin    stMin{};
    STmin    effectiveStMin{};

    N_Result result;
    uint32_t lastRunTime;
    uint8_t  sequenceNumber;
    char*    tag{};

    OSInterface_Mutex* mutex{};
    InternalStatus_t   internalStatus;
    Atomic_int64_t*    availableMemoryForRunners;
    uint32_t           messageOffset;
    int16_t            cfReceivedInThisBlock;

    Timer_N* timerN_Ar{}; // Timer for sending a frame
    Timer_N* timerN_Br{}; // Timer that holds the time since the last FF or CF to the next FC.
    Timer_N* timerN_Cr{}; // Timer that holds the time since the last FC to the next FC.

    OSInterface*        osInterface;
    CANMessageACKQueue* CanMessageACKQueue{};

    CANFrame frameToHold{};
    bool     frameToHoldValid{false};
};

#endif // N_USDATA_INDICATION_RUNNER_H
