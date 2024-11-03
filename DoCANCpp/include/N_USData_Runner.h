#ifndef N_USDATA_RUNNER_H
#define N_USDATA_RUNNER_H

#include "CANShim.h"
#include "OSShim.h"
#include "DoCANCpp_Data_Structures.h"

class N_USData_Runner
{
public:
    typedef enum {RunnerUnknownType, RunnerRequestType, RunnerIndicationType} RunnerType;

    N_USData_Runner(N_AI nAi, OSShim* osShim, CANShim* canShim);

    virtual ~N_USData_Runner() = default;

    // If no frame is received, the runner will only execute if it is not awaiting a message, otherwise it will return an error.
    virtual N_Result run_step(CANFrame* frame) = 0;

    /**
     * @brief Returns if the runner is awaiting a message.
     * @return True if the runner is awaiting a message, false otherwise.
     */
    [[nodiscard]] bool awaitingMessage() const;

    /**
     * @brief Returns the next timestamp the runner will run. The timestamp is derived from OsShim::millis().
     * @return The next timestamp the runner will run.
     */
    [[nodiscard]] uint32_t getNextRunTime() const;

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
     * @brief Returns the minimum separation time between frames of the runner.
     * @return The minimum separation time between frames of the runner.
     */
    [[nodiscard]] uint32_t getSTmin_us() const;

    /**
     * @brief Returns the block size of the runner.
     * @return The block size of the runner.
     */
    [[nodiscard]] uint32_t getBS() const;

    /**
     * @brief Sets the minimum separation time between frames of the runner.
     * @param stMin The minimum separation time between frames of the runner.
     */
    void setSTmin_us(uint32_t stMin);

    /**
     * @brief Sets the block size of the runner.
     * @param blockSize The block size of the runner.
     */
    void setBS(uint32_t blockSize);

    /**
     * @brief Returns the type of the runner.
     * @return The type of the runner.
     */
    [[nodiscard]] RunnerType getRunnerType() const;

    const char* TAG;

protected:
    bool awaitMsg;
    N_AI nAi;
    Mtype mtype;
    uint8_t* messageData;
    uint32_t messageLength;
    uint32_t offset;
    N_Result result;
    uint32_t nextRunTime;
    uint32_t stMin_us;
    uint32_t bs;
    RunnerType runnerType;
    OSShim* osShim;
    CANShim* canShim;
};

#endif //N_USDATA_RUNNER_H
