#ifndef N_USDATA_RUNNER_H
#define N_USDATA_RUNNER_H

#include "CANInterface.h"
#include "CANMessageACKQueue.h"
#include "DoCANCpp_Common.h"
#include "OSInterface.h"

#define NewCANFrameDoCANCpp()                                                                                          \
    {.extd             = 1,                                                                                            \
     .rtr              = 0,                                                                                            \
     .ss               = 0,                                                                                            \
     .self             = 0,                                                                                            \
     .dlc_non_comp     = 0,                                                                                            \
     .reserved         = 0,                                                                                            \
     .identifier       = {.N_AI = 0},                                                                                  \
     .data_length_code = 0,                                                                                            \
     .data             = {0}}

#define returnError(errorCode)                                                                                         \
    do                                                                                                                 \
    {                                                                                                                  \
        internalStatus = ERROR;                                                                                        \
        result         = errorCode;                                                                                    \
        OSInterfaceLogError(tag, "Returning error %s.", N_Result_to_string(errorCode));                                \
        return result;                                                                                                 \
    }                                                                                                                  \
    while (false)

#define returnErrorWithLog(errorCode, fmt, ...)                                                                        \
    do                                                                                                                 \
    {                                                                                                                  \
        internalStatus = ERROR;                                                                                        \
        result         = errorCode;                                                                                    \
        OSInterfaceLogError(tag, "Returning error %s. " fmt, N_Result_to_string(errorCode), ##__VA_ARGS__);            \
        return result;                                                                                                 \
    }                                                                                                                  \
    while (false)

class N_USData_Runner
{
public:
    using RunnerType = enum { RunnerUnknownType, RunnerRequestType, RunnerIndicationType };
    using FrameCode  = enum { SF_CODE = 0b0000, FF_CODE = 0b0001, CF_CODE = 0b0010, FC_CODE = 0b0011 };
    using FlowStatus = enum { CONTINUE_TO_SEND = 0, WAIT = 1, OVERFLOW = 2, INVALID_FS };

    constexpr static uint8_t  MAX_SF_MESSAGE_LENGTH          = 7;
    constexpr static uint8_t  MAX_CF_MESSAGE_LENGTH          = 7;
    constexpr static uint8_t  FC_MESSAGE_LENGTH              = 3;
    constexpr static uint32_t MIN_FF_DL_WITH_ESCAPE_SEQUENCE = 4096;

    constexpr static uint32_t N_As_TIMEOUT_MS = 1000;
    constexpr static uint32_t N_Ar_TIMEOUT_MS = 1000;
    constexpr static uint32_t N_Bs_TIMEOUT_MS = 1000;
    // constexpr static uint32_t N_Br_TIMEOUT_MS = 0.9 * N_Bs_TIMEOUT_MS; // Those are performance requirements.
    constexpr static uint32_t N_Cr_TIMEOUT_MS = 1000;
    // constexpr static uint32_t N_Cs_TIMEOUT_MS = 0.9 * N_Cr_TIMEOUT_MS; // Those are performance requirements.

    N_USData_Runner()          = default;
    virtual ~N_USData_Runner() = default;

    /**
     * @brief Runs the runner.
     *
     * If no frame is received, the runner will only execute if it is not awaiting a message, otherwise it will return
     * an error.
     *
     * @param receivedFrame Pointer to the received frame. If nullptr, no frame is received.
     * @return The result of the run.
     */
    virtual N_Result run_step(CANFrame* receivedFrame) = 0;

    /**
     * @brief Returns if the runner is awaiting a message.
     * @return True if the runner is awaiting a message, false otherwise.
     */
    [[nodiscard]] virtual bool awaitingMessage() const = 0;

    /**
     * @brief Returns the next timestamp the runner will run. The timestamp is derived from OsInterface::millis().
     * @return The next timestamp the runner will run.
     */
    [[nodiscard]] virtual uint32_t getNextRunTime() const = 0;

    /**
     * @brief Returns the N_AI of the runner.
     * @return The N_AI of the runner.
     */
    [[nodiscard]] virtual N_AI getN_AI() const = 0;

    /**
     * @brief Returns the message data of the runner.
     * @return The message data of the runner.
     */
    [[nodiscard]] virtual uint8_t* getMessageData() const = 0;

    /**
     * @brief Returns the message length of the runner.
     * @return The message length of the runner.
     */
    [[nodiscard]] virtual uint32_t getMessageLength() const = 0;

    /**
     * @brief Returns the result of the last run_step().
     * @return The result of the runner.
     */
    [[nodiscard]] virtual N_Result getResult() const = 0;

    /**
     * @brief Returns the Mtype of the runner.
     * @return The Mtype of the runner.
     */
    [[nodiscard]] virtual Mtype getMtype() const = 0;

    /**
     * @brief Returns the type of the runner.
     * @return The type of the runner.
     */
    [[nodiscard]] virtual RunnerType getRunnerType() const = 0;

    /**
     * @brief Callback for when a message is received.
     * @param success True if the message was received successfully, false otherwise.
     */
    virtual void messageACKReceivedCallback(CANInterface::ACKResult success) = 0;
};

#endif // N_USDATA_RUNNER_H
