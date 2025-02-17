#ifndef N_USDATA_RUNNER_H
#define N_USDATA_RUNNER_H

#include "CANInterface.h"
#include "CANMessageACKQueue.h"
#include "DoCANCpp_Data_Structures.h"
#include "OSInterface.h"

#define NewCANFrameDoCANCpp() {.extd = 1, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .reserved = 0, .identifier = {.N_AI = 0}, .data_length_code = 0, .data = {0}}

#define returnError(errorCode)                                                                                                                                                                         \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        internalStatus = ERROR;                                                                                                                                                                        \
        result = errorCode;                                                                                                                                                                            \
        return result;                                                                                                                                                                                 \
    } while (0)

class N_USData_Runner
{
public:
    using RunnerType = enum { RunnerUnknownType, RunnerRequestType, RunnerIndicationType };
    using FrameCode = enum { SF_CODE = 0b0000, FF_CODE = 0b0001, CF_CODE = 0b0010, FC_CODE = 0b0011 };
    using FlowStatus = enum { CONTINUE_TO_SEND = 0, WAIT = 1, OVERFLOW = 2, INVALID_FS };

    constexpr static uint8_t MAX_SF_MESSAGE_LENGTH = 7;
    constexpr static uint32_t MIN_FF_DL_WITH_ESCAPE_SEQUENCE = 4096;

    constexpr static uint32_t N_As_TIMEOUT_MS = 1000;
    constexpr static uint32_t N_Ar_TIMEOUT_MS = 1000;
    constexpr static uint32_t N_Bs_TIMEOUT_MS = 1000;
    // constexpr static uint32_t N_Br_TIMEOUT_MS = 0.9 * N_Bs_TIMEOUT_MS; // Those are performance requirements.
    constexpr static uint32_t N_Cr_TIMEOUT_MS = 1000;
    // constexpr static uint32_t N_Cs_TIMEOUT_MS = 0.9 * N_Cr_TIMEOUT_MS; // Those are performance requirements.

    N_USData_Runner(N_AI nAi, OSInterface& osShim, CANMessageACKQueue& CANmessageACKQueue);

    virtual ~N_USData_Runner();

    // If no frame is received, the runner will only execute if it is not awaiting a message, otherwise it will return an error.
    virtual N_Result run_step(CANFrame* receivedFrame) = 0;

    /**
     * @brief Returns if the runner is awaiting a message.
     * @return True if the runner is awaiting a message, false otherwise.
     */
    [[nodiscard]] virtual bool awaitingMessage() const = 0;

    /**
     * @brief Returns the next timestamp the runner will run. The timestamp is derived from OsShim::millis().
     * @return The next timestamp the runner will run.
     */
    [[nodiscard]] virtual uint32_t getNextRunTime() const = 0;

    /**
     * @brief Returns the N_AI of the runner.
     * @return The N_AI of the runner.
     */
    [[nodiscard]] N_AI getN_AI() const;

    /**
     * @brief Returns the message data of the runner.
     * @return The message data of the runner.
     */
    [[nodiscard]] uint8_t* getMessageData() const;

    /**
     * @brief Returns the message length of the runner.
     * @return The message length of the runner.
     */
    [[nodiscard]] uint32_t getMessageLength() const;

    /**
     * @brief Returns the result of the last run_step().
     * @return The result of the runner.
     */
    [[nodiscard]] N_Result getResult() const;

    /**
     * @brief Returns the Mtype of the runner.
     * @return The Mtype of the runner.
     */
    [[nodiscard]] Mtype getMtype() const;

    /**
     * @brief Returns the type of the runner.
     * @return The type of the runner.
     */
    [[nodiscard]] RunnerType getRunnerType() const;

    /**
     * @brief Callback for when a message is received.
     * @param success True if the message was received successfully, false otherwise.
     */
    virtual void messageACKReceivedCallback(CANInterface::ACKResult success) = 0;

    const char* TAG;

protected:
    static uint32_t getStMinInMs(STmin stMin);
    virtual N_Result checkTimeouts() = 0;

    OSInterface_Mutex* mutex;
    N_AI nAi;
    Mtype mType;
    uint8_t* messageData;
    int64_t messageLength;
    N_Result result;
    RunnerType runnerType;
    OSInterface* osShim;
    CANMessageACKQueue* CANmessageACKQueue;
    uint8_t blockSize;
    uint32_t lastRunTime;
    uint8_t sequenceNumber;
    STmin stMin{};
};

#endif // N_USDATA_RUNNER_H
