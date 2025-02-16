#include "N_USData_Runner.h"

#include <cassert>

N_USData_Runner::N_USData_Runner(N_AI nAi, OSShim& osShim, CANMessageACKQueue& CANmessageACKQueue)
{
    this->nAi = nAi;
    this->mType = Mtype_Unknown;
    this->messageData = nullptr;
    this->messageLength = 0;
    this->result = NOT_STARTED;
    this->runnerType = RunnerUnknownType;
    this->osShim = &osShim;
    this->CANmessageACKQueue = &CANmessageACKQueue;
    this->blockSize = 0;
    this->stMin = {0, ms};
    this->lastRunTime = 0;
    this->sequenceNumber = 1;

    this->mutex = osShim.osCreateMutex();
    assert(this->mutex != nullptr && "Failed to create mutex");

    this->TAG = nullptr;
}

N_USData_Runner::~N_USData_Runner()
{
    delete mutex;
}

uint32_t N_USData_Runner::getStMinInMs(STmin stMin)
{
    return stMin.unit == usX100 ? 1 : stMin.value; // 1 ms is the smallest resolution we can get.
}

N_AI N_USData_Runner::getN_AI() const { return nAi; }

uint8_t* N_USData_Runner::getMessageData() const { return messageData; }

uint32_t N_USData_Runner::getMessageLength() const { return messageLength; }

N_Result N_USData_Runner::getResult() const { return result; }

Mtype N_USData_Runner::getMtype() const { return mType; }

N_USData_Runner::RunnerType N_USData_Runner::getRunnerType() const { return this->runnerType; }
