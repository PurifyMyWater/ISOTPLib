#include "N_USData_Indication_Runner.h"

#include <cassert>
#include <cstring>

N_USData_Indication_Runner::N_USData_Indication_Runner(N_AI nAi, Atomic_int64_t& availableMemoryForRunners,
                                                       uint8_t blockSize, STmin stMin, OSInterface& osInterface,
                                                       CANMessageACKQueue& canMessageACKQueue)
{
    this->TAG = "DoCANCpp_IndicationRunner";

    this->mType              = Mtype_Unknown;
    this->osInterface        = &osInterface;
    this->CanMessageACKQueue = &canMessageACKQueue;
    this->messageData        = nullptr;
    this->messageLength      = 0;
    this->result             = NOT_STARTED;
    this->lastRunTime        = 0;
    this->sequenceNumber = 1; // The first sequence number that is being sent is 1. (0 is reserved for the first frame)

    this->mutex = osInterface.osCreateMutex();
    assert(this->mutex != nullptr && "Failed to create mutex");

    this->internalStatus            = NOT_RUNNING;
    this->nAi                       = nAi;
    this->stMin                     = stMin;
    this->blockSize                 = blockSize;
    this->effectiveBlockSize        = blockSize;
    this->effectiveStMin            = stMin;
    this->availableMemoryForRunners = &availableMemoryForRunners;
    this->osInterface               = &osInterface;
    this->messageData               = nullptr;
    this->messageOffset             = 0;
    this->cfReceivedInThisBlock     = 0;

    this->timerN_Ar = new Timer_N(osInterface);
    this->timerN_Br = new Timer_N(osInterface);
    this->timerN_Cr = new Timer_N(osInterface);
}

N_USData_Indication_Runner::~N_USData_Indication_Runner()
{
    if (this->messageData != nullptr)
    {
        this->osInterface->osFree(messageData);
        this->availableMemoryForRunners->add(this->messageLength * static_cast<int64_t>(sizeof(uint8_t)));
    }

    delete mutex;
}

bool N_USData_Indication_Runner::setBlockSize(uint8_t blockSize)
{
    if (mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        this->blockSize = blockSize;
        mutex->signal();
        return true;
    }
    return false;
}

bool N_USData_Indication_Runner::setSTmin(STmin stMin)
{
    if (mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        this->stMin = stMin;
        mutex->signal();
        return true;
    }
    return false;
}

N_Result N_USData_Indication_Runner::run_step_notRunning(const CANFrame* receivedFrame)
{
    if (receivedFrame == nullptr)
    {
        returnError(N_ERROR);
    }

    if (receivedFrame->identifier.N_TAtype != N_TATYPE_5_CAN_CLASSIC_29bit_Physical &&
        receivedFrame->identifier.N_TAtype != N_TATYPE_6_CAN_CLASSIC_29bit_Functional)
    {
        returnError(N_ERROR); // The frame is not a Mtype_Diagnostics frame.
    }
    this->mType = Mtype_Diagnostics; // We check if the frame is a diagnostics frame by looking at the N_TAType. (205 &
                                     // 206 is the value used for remote diagnostics)

    switch (receivedFrame->data[0] >> 4)
    {
        case SF_CODE:
        {
            messageLength = receivedFrame->data[0] & 0b00001111;

            if (messageLength <= MAX_SF_MESSAGE_LENGTH &&
                this->availableMemoryForRunners->subIfResIsGreaterThanZero(this->messageLength *
                                                                           static_cast<int64_t>(sizeof(uint8_t))))
            {
                messageData = static_cast<uint8_t*>(osInterface->osMalloc(this->messageLength * sizeof(uint8_t)));
                memcpy(messageData, &receivedFrame->data[1], messageLength);
                result = N_OK;
                return result;
            }

            returnError(N_ERROR);
        }
        case FF_CODE:
        {
            timerN_Br->startTimer();
            if (nAi.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional)
            {
                returnError(N_UNEXP_PDU);
            }

            messageLength =
                (receivedFrame->data[0] & 0b00001111) << 8 |
                receivedFrame
                    ->data[1];      // unpack the message length (12 bits) 4 in data[0] lower 4 bits and 8 in data[1]
            if (messageLength == 0) // Escape sequence -> length is >= MIN_FF_DL_WITH_ESCAPE_SEQUENCE
            {
                messageLength = receivedFrame->data[2] << 24 | receivedFrame->data[3] << 16 |
                                receivedFrame->data[4] << 8 |
                                receivedFrame->data[5]; // unpack the message length (32 bits) 8 in data[2], 8 in
                                                        // data[3], 8 in data[4] and 8 in data[5]
            }

            if (messageLength <= MAX_SF_MESSAGE_LENGTH)
            {
                returnError(N_ERROR);
            }

            if (availableMemoryForRunners->subIfResIsGreaterThanZero(
                    this->messageLength * static_cast<int64_t>(sizeof(uint8_t)))) // Check if there is enough memory
            {
                this->messageData = static_cast<uint8_t*>(osInterface->osMalloc(messageLength * sizeof(uint8_t)));

                if (this->messageData == nullptr)
                {
                    returnError(N_ERROR);
                }

                if (messageLength < MIN_FF_DL_WITH_ESCAPE_SEQUENCE)
                {
                    memcpy(messageData, &receivedFrame->data[2], 6);
                    messageOffset = 6;
                }
                else
                {
                    memcpy(messageData, &receivedFrame->data[6], 2);
                    messageOffset = 2;
                }

                timerN_Br->stopTimer();

                if (sendFCFrame(CONTINUE_TO_SEND) != N_OK)
                {
                    returnError(N_ERROR);
                }

                timerN_Ar->startTimer();

                result = IN_PROGRESS_FF;
                return result;
            }
            sendFCFrame(OVERFLOW);
            returnError(N_ERROR);
        }
        default:
            returnError(N_UNEXP_PDU);
    }
}

N_Result N_USData_Indication_Runner::sendFCFrame(FlowStatus fs)
{
    effectiveBlockSize = blockSize;
    effectiveStMin     = stMin;

    CANFrame fcFrame            = NewCANFrameDoCANCpp();
    fcFrame.identifier.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical;
    fcFrame.identifier.N_TA     = nAi.N_SA;
    fcFrame.identifier.N_SA     = nAi.N_TA;

    fcFrame.data[0] = FC_CODE << 4 | fs;
    fcFrame.data[1] = effectiveBlockSize;

    if (effectiveStMin.unit == ms)
    {
        fcFrame.data[2] = effectiveStMin.value;
    }
    else
    {
        fcFrame.data[2] = 0b11110000 | effectiveStMin.value;
    }

    fcFrame.data_length_code = FC_MESSAGE_LENGTH;

    if (CanMessageACKQueue->writeFrame(*this, fcFrame))
    {
        internalStatus = AWAITING_FC_ACK;
        return N_OK;
    }

    return N_ERROR;
}

N_Result N_USData_Indication_Runner::run_step_CF(const CANFrame* receivedFrame)
{
    if (receivedFrame == nullptr)
    {
        returnError(N_ERROR);
    }

    if (nAi.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional)
    {
        returnError(N_UNEXP_PDU);
    }

    if (receivedFrame->data[0] >> 4 != CF_CODE)
    {
        returnError(N_UNEXP_PDU);
    }

    if ((receivedFrame->data[0] & 0b00001111) != sequenceNumber)
    {
        returnError(N_WRONG_SN);
    }

    sequenceNumber++;

    if (receivedFrame->data_length_code <= 1)
    {
        returnError(N_ERROR);
    }

    uint8_t bytesToCopy =
        MIN(receivedFrame->data_length_code - 1,
            messageLength - messageOffset); // Copy the minimum between the remaining bytes and the received bytes (1st
                                            // byte is used to transport metadata).

    memcpy(&messageData[messageOffset], &receivedFrame->data[1], bytesToCopy);

    messageOffset += bytesToCopy;
    cfReceivedInThisBlock++;

    if (messageOffset == messageLength)
    {
        timerN_Cr->stopTimer();
        result = N_OK;
    }
    else
    {
        if (effectiveBlockSize == cfReceivedInThisBlock)
        {
            timerN_Cr->stopTimer();
            timerN_Br->startTimer();
            if (sendFCFrame(CONTINUE_TO_SEND) != N_OK)
            {
                returnError(N_ERROR);
            }
            timerN_Ar->startTimer();
            cfReceivedInThisBlock = 0;
        }
        else
        {
            timerN_Cr->startTimer();
        }
        result = IN_PROGRESS;
    }
    return result;
}

N_Result N_USData_Indication_Runner::checkTimeouts()
{
#if !DOCANCPP_DISABLE_TIMEOUTS

    if (timerN_Ar->getElapsedTime_ms() > N_Ar_TIMEOUT_MS)
    {
        returnError(N_TIMEOUT_A);
    }
    if (timerN_Cr->getElapsedTime_ms() > N_Cr_TIMEOUT_MS)
    {
        returnError(N_TIMEOUT_Cr);
    }
#endif
    return N_OK;
}

N_Result N_USData_Indication_Runner::run_step(CANFrame* receivedFrame)
{
    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        result         = N_ERROR;
        internalStatus = ERROR;
        return result;
    }

    N_Result res = checkTimeouts();

    if (res != N_OK)
    {
        mutex->signal();
        return res;
    }

    switch (internalStatus)
    {
        case NOT_RUNNING:
            res = run_step_notRunning(receivedFrame);
            break;
        case AWAITING_CF:
            res = run_step_CF(receivedFrame);
            break;
        case ERROR:
            res = result;
            break;
        default:
            assert(false && "Invalid internal status");
    }

    lastRunTime = osInterface->osMillis();
    mutex->signal();
    return res;
}

bool N_USData_Indication_Runner::awaitingMessage() const
{
    return internalStatus == NOT_RUNNING || internalStatus == AWAITING_CF;
}

uint32_t N_USData_Indication_Runner::getNextTimeoutTime() const
{
    uint32_t timeoutAr = timerN_Ar->getStartTimeStamp() + N_Ar_TIMEOUT_MS;
    uint32_t timeoutCr = timerN_Cr->getStartTimeStamp() + N_Cr_TIMEOUT_MS;
    return MIN(timeoutAr, timeoutCr);
}

uint32_t N_USData_Indication_Runner::getNextRunTime() const
{
    uint32_t nextRunTime = getNextTimeoutTime();
    switch (internalStatus)
    {
        case NOT_RUNNING:
            nextRunTime = 0; // Execute as soon as possible
            break;
        default:
            break;
    }

    return nextRunTime;
}

void N_USData_Indication_Runner::messageACKReceivedCallback(CANInterface::ACKResult success)
{
    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        result         = N_ERROR;
        internalStatus = ERROR;
        return;
    }

    switch (internalStatus)
    {
        case AWAITING_FC_ACK:
        {
            if (success == CANInterface::ACK_SUCCESS)
            {
                timerN_Ar->stopTimer();
                timerN_Cr->startTimer();
                this->internalStatus = AWAITING_CF;
            }
            else
            {
                result         = N_ERROR;
                internalStatus = ERROR;
            }
        }
        break;
        default:
            assert(false && "Invalid internal status");
    }

    mutex->signal();
}

N_AI N_USData_Indication_Runner::getN_AI() const
{
    return nAi;
}

uint8_t* N_USData_Indication_Runner::getMessageData() const
{
    return messageData;
}

uint32_t N_USData_Indication_Runner::getMessageLength() const
{
    return messageLength;
}

N_Result N_USData_Indication_Runner::getResult() const
{
    return result;
}

Mtype N_USData_Indication_Runner::getMtype() const
{
    return mType;
}

N_USData_Indication_Runner::RunnerType N_USData_Indication_Runner::getRunnerType() const
{
    return RunnerIndicationType;
}
