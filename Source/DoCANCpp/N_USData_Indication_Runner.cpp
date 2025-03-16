#include "N_USData_Indication_Runner.h"

#include <cassert>
#include <cstring>

// TODO log the creation of the runner
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
    assert(this->mutex != nullptr && "Failed to create mutex"); // TODO use result variable

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
    OSInterfaceLogDebug(tag, "Deleting runner");
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
        OSInterfaceLogInfo(tag, "Block size set to %d", blockSize);
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
        OSInterfaceLogInfo(tag, "STmin set to %d%s", stMin.value, stMin.unit == ms ? " ms" : "00 us");
        return true;
    }
    return false;
}

N_Result N_USData_Indication_Runner::run_step_notRunning(const CANFrame* receivedFrame)
{
    if (receivedFrame == nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is null");
    }

    if (receivedFrame->identifier.N_TAtype != N_TATYPE_5_CAN_CLASSIC_29bit_Physical &&
        receivedFrame->identifier.N_TAtype != N_TATYPE_6_CAN_CLASSIC_29bit_Functional)
    {
        returnErrorWithLog(N_ERROR,
                           "The frame is not a Mtype_Diagnostics frame"); // The frame is not a Mtype_Diagnostics frame.
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

                OSInterfaceLogInfo(tag, "Received message with length %ld (SF)", messageLength);
                result = N_OK;
                return result;
            }

            int64_t availableMemory;
            availableMemoryForRunners->get(&availableMemory);
            returnErrorWithLog(N_ERROR, "Message length is %ld and available memory is %ld", messageLength,
                               availableMemory);
        }
        case FF_CODE:
        {
            timerN_Br->startTimer();
            if (nAi.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional)
            {
                returnErrorWithLog(N_UNEXP_PDU, "Received FF frame with N_TAtype %d", nAi.N_TAtype);
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
                returnErrorWithLog(N_ERROR, "FF frame with length %ld is too small", messageLength);
            }

            OSInterfaceLogDebug(tag, "Received FF frame with length %ld", messageLength);

            int64_t availableMemory;
            availableMemoryForRunners->get(&availableMemory);

            if (availableMemoryForRunners->subIfResIsGreaterThanZero(
                    this->messageLength * static_cast<int64_t>(sizeof(uint8_t)))) // Check if there is enough memory
            {
                this->messageData = static_cast<uint8_t*>(osInterface->osMalloc(messageLength * sizeof(uint8_t)));

                if (this->messageData == nullptr)
                {
                    returnErrorWithLog(N_ERROR, "Not enough memory for message length %ld. Available memory is %ld", messageLength,
                                availableMemory);
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
                OSInterfaceLogVerbose(tag, "Timer N_Br stopped after receiving FF frame in %u ms", timerN_Br->getElapsedTime_ms());

                if (sendFCFrame(CONTINUE_TO_SEND) != N_OK)
                {
                    returnErrorWithLog(N_ERROR, "Flow control frame could not be sent");
                }

                timerN_Ar->startTimer();
                OSInterfaceLogVerbose(tag, "Timer N_Ar started after sending FC frame in %u ms", timerN_Ar->getElapsedTime_ms());

                result = IN_PROGRESS_FF;
                return result;
            }
            sendFCFrame(OVERFLOW);
            returnErrorWithLog(N_ERROR, "Not enough memory for message length %ld. Available memory is %ld", messageLength,
                                availableMemory);
        }
        default:
            returnErrorWithLog(N_UNEXP_PDU, "Received frame with invalid PDU code %d", receivedFrame->data[0] >> 4);
    }
}

N_Result N_USData_Indication_Runner::sendFCFrame(const FlowStatus fs)
{
    effectiveBlockSize = blockSize;
    effectiveStMin     = stMin;

    CANFrame fcFrame            = NewCANFrameDoCANCpp();
    fcFrame.identifier.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical;
    fcFrame.identifier.N_TA     = nAi.N_SA;
    fcFrame.identifier.N_SA     = nAi.N_TA;

    fcFrame.data[0] = FC_CODE << 4 | fs;
    fcFrame.data[1] = effectiveBlockSize; // Only relevant if fs == CONTINUE_TO_SEND, otherwise ignored.

    if (effectiveStMin.unit == ms) // Only relevant if fs == CONTINUE_TO_SEND, otherwise ignored.
    {
        fcFrame.data[2] = effectiveStMin.value;
    }
    else
    {
        fcFrame.data[2] = 0b11110000 | effectiveStMin.value;
    }

    fcFrame.data_length_code = FC_MESSAGE_LENGTH;

    OSInterfaceLogDebug(tag, "Sending FC frame with flow status %d, block size %d and STmin %d%s", fs,
                        effectiveBlockSize, effectiveStMin.value, effectiveStMin.unit == ms ? " ms" : "00 us");

    if (CanMessageACKQueue->writeFrame(*this, fcFrame))
    {
        internalStatus = AWAITING_FC_ACK;
        return N_OK;
    }

    OSInterfaceLogError(tag, "FC frame could not be sent");
    return N_ERROR;
}

N_Result N_USData_Indication_Runner::run_step_CF(const CANFrame* receivedFrame)
{
    if (receivedFrame == nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is null");
    }

    if (nAi.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional)
    {
        returnErrorWithLog(N_UNEXP_PDU, "Received CF frame with N_TAtype %d", nAi.N_TAtype);
    }

    if (receivedFrame->data[0] >> 4 != CF_CODE)
    {
        returnErrorWithLog(N_UNEXP_PDU, "Received frame is not a CF frame");
    }

    uint8_t messageSequenceNumber = (receivedFrame->data[0] & 0b00001111);
    if (messageSequenceNumber != sequenceNumber)
    {
        returnErrorWithLog(N_WRONG_SN, "Received CF frame with wrong sequence number %d. Was expecting %d", messageSequenceNumber, sequenceNumber);
    }

    sequenceNumber++;

    if (receivedFrame->data_length_code <= 1)
    {
        returnErrorWithLog(N_ERROR, "Received CF frame with invalid data length code %d", receivedFrame->data_length_code);
    }

    uint8_t bytesToCopy =
        MIN(receivedFrame->data_length_code - 1,
            messageLength - messageOffset); // Copy the minimum between the remaining bytes and the received bytes (1st
                                            // byte is used to transport metadata).

    memcpy(&messageData[messageOffset], &receivedFrame->data[1], bytesToCopy);

    messageOffset += bytesToCopy;
    cfReceivedInThisBlock++;

    OSInterfaceLogDebug(tag, "Received CF #%d in block with %d data bytes", cfReceivedInThisBlock, bytesToCopy);

    if (messageOffset == messageLength)
    {
        timerN_Cr->stopTimer();
        OSInterfaceLogVerbose(tag, "Timer N_Cr stopped after receiving CF frame in %u ms", timerN_Cr->getElapsedTime_ms());
        result = N_OK;
    }
    else
    {
        if (effectiveBlockSize == cfReceivedInThisBlock)
        {
            timerN_Cr->stopTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Cr stopped after receiving CF frame in %u ms", timerN_Cr->getElapsedTime_ms());
            timerN_Br->startTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Br started after receiving CF frame in %u ms", timerN_Br->getElapsedTime_ms());

            if (sendFCFrame(CONTINUE_TO_SEND) != N_OK)
            {
                returnErrorWithLog(N_ERROR, "Flow control frame could not be sent");
            }
            timerN_Ar->startTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Ar started after sending FC frame in %u ms", timerN_Ar->getElapsedTime_ms());
            cfReceivedInThisBlock = 0;

            OSInterfaceLogDebug(tag, "CF block size reached. Waiting for FC frame");
        }
        else
        {
            timerN_Cr->startTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Cr started after receiving CF frame in %u ms", timerN_Cr->getElapsedTime_ms());
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
        returnErrorWithLog(N_TIMEOUT_A, "Elapsed time is %u ms", timerN_Ar->getElapsedTime_ms());
    }
    if (timerN_Cr->getElapsedTime_ms() > N_Cr_TIMEOUT_MS)
    {
        returnErrorWithLog(N_TIMEOUT_Cr, "Elapsed time is %u ms", timerN_Cr->getElapsedTime_ms());
    }
#endif
    return N_OK;
}

N_Result N_USData_Indication_Runner::run_step(CANFrame* receivedFrame)
{
    OSInterfaceLogVerbose(tag, "Running step with frame %s",
                          receivedFrame != nullptr ? frameToString(*receivedFrame) : "null");
    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        returnErrorWithLog(N_ERROR, "Failed to acquire mutex");
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
            OSInterfaceLogError(tag, "Invalid internal status %d", internalStatus);
            result         = N_ERROR;
            internalStatus = ERROR;
            res            = result;
            break;
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
    uint32_t minTimeout = MIN(timeoutAr, timeoutCr);

    OSInterfaceLogVerbose(tag, "Next timeout is in %u ms", minTimeout - osInterface->osMillis());
    return minTimeout;
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

    OSInterfaceLogDebug(tag, "Next run time is in %u ms", nextRunTime - osInterface->osMillis());
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
                OSInterfaceLogDebug(tag, "FC ACK received");
            }
            else
            {
                OSInterfaceLogError(tag, "FC ACK failed with result %d", success);
                result         = N_ERROR;
                internalStatus = ERROR;
            }
        }
        break;
        default:
            OSInterfaceLogError(tag, "Invalid internal status %d", internalStatus);
            result         = N_ERROR;
            internalStatus = ERROR;
            break;
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
