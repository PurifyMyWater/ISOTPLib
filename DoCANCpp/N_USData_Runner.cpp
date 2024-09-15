#include "N_USData_Runner.h"

N_USData_Runner::N_USData_Runner(N_AI nAi, OSShim* osShim, CANShim* canShim)
{
    this->awaitMsg = false;
    this->nAi = nAi;
    this->mtype = Mtype::Mtype_Unknown;
    this->messageData = nullptr;
    this->messageLength = 0;
    this->availableMemoryForRunners = nullptr;
    this->offset = 0;
    this->result = N_Result::NOT_STARTED;
    this->nextRunTime = 0;
    this->stMin_us = 0;
    this->bs = 0;
    this->runnerType = RunnerUnknownType;
    this->osShim = osShim;
    this->canShim = canShim;

    this->TAG = nullptr;
}

bool N_USData_Runner::awaitingMessage() const
{
    return awaitMsg;
}

uint32_t N_USData_Runner::getNextRunTime() const
{
    return nextRunTime;
}

N_AI N_USData_Runner::getN_AI() const
{
    return nAi;
}

uint8_t* N_USData_Runner::getMessageData() const
{
    return messageData;
}

uint32_t N_USData_Runner::getMessageLength() const
{
    return messageLength;
}

N_Result N_USData_Runner::getResult() const
{
    return result;
}

Mtype N_USData_Runner::getMtype() const
{
    return mtype;
}

uint32_t N_USData_Runner::getSTmin_us() const
{
    return stMin_us;
}

uint32_t N_USData_Runner::getBS() const
{
    return bs;
}

void N_USData_Runner::setSTmin_us(uint32_t stMin)
{
    this->stMin_us = stMin;
}

void N_USData_Runner::setBS(uint32_t blockSize)
{
    this->bs = blockSize;
}

N_USData_Runner::RunnerType N_USData_Runner::getRunnerType() const
{
    return this->runnerType;
}
