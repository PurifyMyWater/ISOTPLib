#ifndef N_USDATA_REQUEST_RUNNER_H
#define N_USDATA_REQUEST_RUNNER_H

#include "Atomic_int64_t.h"
#include "CANMessageACKQueue.h"
#include "N_USData_Runner.h"
#include "Timer_N.h"

constexpr char    N_USDATA_REQUEST_RUNNER_STATIC_TAG[] = "DoCANCpp_RequestRunner_";
constexpr int32_t N_USDATA_REQUEST_RUNNER_TAG_SIZE     = MAX_N_AI_STR_SIZE + sizeof(N_USDATA_REQUEST_RUNNER_STATIC_TAG);

// Class that handles the request aka transmission of a message
class N_USData_Request_Runner : public N_USData_Runner
{
public:
    N_USData_Request_Runner(bool& result, N_AI nAi, Atomic_int64_t& availableMemoryForRunners, Mtype mType,
                            const uint8_t* messageData, uint32_t messageLength, OSInterface& osInterface,
                            CANMessageACKQueue& canMessageACKQueue);

    ~N_USData_Request_Runner() override;

    N_Result runStep(CANFrame* receivedFrame) override;

    [[nodiscard]] uint32_t getNextRunTime() override;

    void messageACKReceivedCallback(CANInterface::ACKResult success) override;

    [[nodiscard]] N_AI getN_AI() const override;

    [[nodiscard]] uint8_t* getMessageData() const override;

    [[nodiscard]] uint32_t getMessageLength() const override;

    [[nodiscard]] N_Result getResult() const override;

    [[nodiscard]] Mtype getMtype() const override;

    [[nodiscard]] RunnerType getRunnerType() const override;

    [[nodiscard]] const char* getTAG() const override;

    [[nodiscard]] bool isThisFrameForMe(const CANFrame& frame) const override;

private:
    N_Result runStep_holdFrame(const CANFrame* receivedFrame);
    N_Result runStep_internal(const CANFrame* receivedFrame);
    N_Result runStep_SF(const CANFrame* receivedFrame);
    N_Result runStep_FF(const CANFrame* receivedFrame);
    N_Result runStep_CF(const CANFrame* receivedFrame);
    N_Result runStep_FC(const CANFrame* receivedFrame, bool firstFc = false);

    void SF_ACKReceivedCallback(CANInterface::ACKResult success);
    void FF_ACKReceivedCallback(CANInterface::ACKResult success);
    void CF_ACKReceivedCallback(CANInterface::ACKResult success);

    N_Result               parseFCFrame(const CANFrame* receivedFrame, FlowStatus& fs, uint8_t& blcksize, STmin& stM);
    [[nodiscard]] uint32_t getNextTimeoutTime() const;
    N_Result               checkTimeouts();
    N_Result               sendCFFrame();
    [[nodiscard]] bool     awaitingFrame(const CANFrame& frame) const;

    using InternalStatus_t = enum {
        NOT_RUNNING_SF,
        AWAITING_SF_ACK,
        NOT_RUNNING_FF,
        AWAITING_FF_ACK,
        AWAITING_FirstFC,
        AWAITING_FC,
        SEND_CF,
        AWAITING_CF_ACK,
        MESSAGE_SENT,
        ERROR
    };

    [[nodiscard]] static const char* internalStatusToString(InternalStatus_t status);

    N_AI     nAi;
    Mtype    mType;
    uint8_t* messageData{};
    int64_t  messageLength;
    uint8_t  blockSize;
    STmin    stMin{};

    N_Result        result;
    uint32_t        lastRunTime;
    uint8_t         sequenceNumber;
    Atomic_int64_t* availableMemoryForRunners;
    uint32_t        messageOffset;
    char*           tag{};

    OSInterface_Mutex* mutex{};
    InternalStatus_t   internalStatus;
    int16_t            cfSentInThisBlock;

    Timer_N* timerN_As{}; // Timer for sending a frame
    Timer_N* timerN_Bs{}; // Timer that holds the time since the last FF or CF to the next CF.
    Timer_N* timerN_Cs{}; // Timer that calls out once STmin has passed.

    OSInterface*        osInterface;
    CANMessageACKQueue* CanMessageACKQueue;

    CANFrame frameToHold{};
    bool     frameToHoldValid{false};
};

#endif // N_USDATA_REQUEST_RUNNER_H
