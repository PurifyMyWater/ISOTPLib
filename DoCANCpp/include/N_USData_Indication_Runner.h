#ifndef N_USDATA_INDICATION_RUNNER_H
#define N_USDATA_INDICATION_RUNNER_H

#include "N_USData_Runner.h"
#include "DoCANCpp.h"

#include "Atomic_int64_t.h"

// Class that handles the indication aka reception of a message
class N_USData_Indication_Runner : public N_USData_Runner
{
public:
    N_USData_Indication_Runner(N_AI nAi, Atomic_int64_t& availableMemoryForRunners, uint8_t blockSize, STmin stMin, OSShim& osShim, CANShim& canShim);

    ~N_USData_Indication_Runner() override;

    N_Result run_step(CANFrame* frame) override;

    [[nodiscard]] bool awaitingMessage() const override;

    [[nodiscard]] uint32_t getNextRunTime() const override;
private:
    Atomic_int64_t* availableMemoryForRunners;
    uint8_t blockSize;
    STmin stMin;
};

#endif //N_USDATA_INDICATION_RUNNER_H
