#ifndef N_USDATA_RUNNER_H
#define N_USDATA_RUNNER_H

#include "CANShim.h"
#include "DoCANCpp_Data_Structures.h"
#include "OSShim.h"

#define NewCANFrameDoCANCpp() {.extd = 1, .rtr = 0, .ss = 0, .self = 0, .dlc_non_comp = 0, .reserved = 0, .identifier = {.N_AI = 0}, .data_length_code = 0, .data = {0}}

class N_USData_Runner
{
public:
    using RunnerType = enum { RunnerUnknownType, RunnerRequestType, RunnerIndicationType };
    using FrameCode = enum { SF_CODE = 0b0000, FF_CODE = 0b0001, CF_CODE = 0b0010, FC_CODE = 0b0011 };

    constexpr static uint8_t MAX_SF_MESSAGE_LENGTH = 7;
    constexpr static uint8_t MIN_FF_DL_WITH_ESCAPE_SEQUENCE = 4096;

    N_USData_Runner(N_AI nAi, OSShim& osShim, CANShim& canShim);

    virtual ~N_USData_Runner() = default;

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

    const char* TAG;

protected:
    N_AI nAi;
    Mtype mType;
    uint8_t* messageData;
    int64_t messageLength;
    N_Result result;
    RunnerType runnerType;
    OSShim* osShim;
    CANShim* canShim;
};

#endif // N_USDATA_RUNNER_H
