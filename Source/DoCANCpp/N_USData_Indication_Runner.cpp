#include "N_USData_Indication_Runner.h"
#include <cassert>
#include <cstring>

N_USData_Indication_Runner::N_USData_Indication_Runner(bool& result, const N_AI nAi,
                                                       Atomic_int64_t& availableMemoryForRunners,
                                                       const uint8_t blockSize, const STmin stMin,
                                                       OSInterface& osInterface, CANMessageACKQueue& canMessageACKQueue)
{
    result = false;

    this->availableMemoryForRunners = &availableMemoryForRunners;
    this->osInterface               = &osInterface;

    if (this->availableMemoryForRunners->subIfResIsGreaterThanZero(N_USDATA_INDICATION_RUNNER_TAG_SIZE))
    {
        this->tag = static_cast<char*>(this->osInterface->osMalloc(N_USDATA_INDICATION_RUNNER_TAG_SIZE));
        if (this->tag == nullptr)
        {
            return;
        }
        snprintf(this->tag, N_USDATA_INDICATION_RUNNER_TAG_SIZE, "%s%s", N_USDATA_INDICATION_RUNNER_STATIC_TAG,
                 nAiToString(nAi));
    }
    else
    {
        return;
    }

    OSInterfaceLogDebug(tag, "Creating N_USData_Indication_Runner with tag %s", this->tag);

    this->mType              = Mtype_Unknown;
    this->CanMessageACKQueue = &canMessageACKQueue;
    this->messageData        = nullptr;
    this->messageLength      = 0;
    this->result             = NOT_STARTED;
    this->lastRunTime        = 0;
    this->sequenceNumber = 1; // The first sequence number that is being sent is 1. (0 is reserved for the first frame)

    this->mutex = osInterface.osCreateMutex();
    if (this->mutex == nullptr)
    {
        OSInterfaceLogError(tag, AT "Failed to create mutex");
        return;
    }

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

    result = true;
}

// Be careful with the destructor. All the pointers used in the destructor need to be initialized to nullptr. Otherwise,
// the destructor may attempt a free on an invalid pointer.
N_USData_Indication_Runner::~N_USData_Indication_Runner()
{
    OSInterfaceLogDebug(tag, "Deleting runner");

    if (this->messageData != nullptr)
    {
        osInterface->osFree(this->messageData);
        availableMemoryForRunners->add(messageLength * static_cast<int64_t>(sizeof(uint8_t)));
    }

    if (this->tag != nullptr)
    {
        osInterface->osFree(this->tag);
        availableMemoryForRunners->add(N_USDATA_INDICATION_RUNNER_TAG_SIZE);
    }

    delete timerN_Ar;
    delete timerN_Br;
    delete timerN_Cr;
    delete mutex;
}

N_Result N_USData_Indication_Runner::runStep(CANFrame* receivedFrame)
{
    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        returnErrorWithLog(N_ERROR, "Failed to acquire mutex");
    }

    OSInterfaceLogVerbose(tag, "Running step with internalStatus = %s (%d) and frame %s",
                          internalStatusToString(internalStatus), internalStatus,
                          receivedFrame != nullptr ? frameToString(*receivedFrame) : "null");

    N_Result res = checkTimeouts();

    if (res != N_OK)
    {
        mutex->signal();
        return res;
    }

    res = runStep_internal(receivedFrame);

    mutex->signal();
    return res;
}

N_Result N_USData_Indication_Runner::runStep_internal(CANFrame* receivedFrame)
{
    N_Result res;

    switch (internalStatus)
    {
        case NOT_RUNNING:
            res = runStep_notRunning(receivedFrame);
            break;
        case SEND_FC:
            res = runStep_FC_CTS(receivedFrame);
            break;
        case AWAITING_CF:
            res = runStep_CF(receivedFrame);
            break;
        case AWAITING_FC_ACK:
            res = runStep_holdFrame(receivedFrame);
            break;
        case MESSAGE_RECEIVED:
            OSInterfaceLogDebug(tag, "Message received successfully");
            result =
                N_OK; // If the message is successfully received, return N_OK to allow DoCanCpp to call the callback.
            res = result;
            break;
        case ERROR:
            res = result;
            break;
        default:
            OSInterfaceLogError(tag, "Invalid internalStatus %s (%d)", internalStatusToString(internalStatus),
                                internalStatus);
            result = N_ERROR;
            updateInternalStatus(ERROR);
            res = result;
            break;
    }

    lastRunTime = osInterface->osMillis();

    return res;
}

N_Result N_USData_Indication_Runner::runStep_holdFrame(const CANFrame* receivedFrame)
{
    if (receivedFrame == nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is null");
    }

    OSInterfaceLogWarning(tag, "Received frame while waiting for ACK. Storing it for later use Frame: %s",
                          frameToString(*receivedFrame));

    frameToHold      = *receivedFrame; // Store the frame for later use.
    frameToHoldValid = true;           // Mark the frame as valid.

    result = IN_PROGRESS; // Indicate that we are still waiting for the ACK.
    return result;
}

N_Result N_USData_Indication_Runner::runStep_notRunning(const CANFrame* receivedFrame)
{
    if (receivedFrame == nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is null");
    }

    if (receivedFrame->identifier.N_TAtype != N_TATYPE_5_CAN_CLASSIC_29bit_Physical &&
        receivedFrame->identifier.N_TAtype != N_TATYPE_6_CAN_CLASSIC_29bit_Functional)
    {
        returnErrorWithLog(N_ERROR, "The frame is not a Mtype_Diagnostics frame (%d)",
                           receivedFrame->identifier.N_TAtype); // The frame is not a Mtype_Diagnostics frame.
    }
    this->mType = Mtype_Diagnostics; // We check if the frame is a diagnostics frame by looking at the N_TAType. (205 &
                                     // 206 is the value used for remote diagnostics)

    switch (FrameCode frameCode = static_cast<FrameCode>(receivedFrame->data[0] >> 4))
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
            returnErrorWithLog(N_ERROR, "Not enough memory for message length %ld. Available memory is %ld",
                               messageLength, availableMemory);
        }
        case FF_CODE:
        {
            timerN_Br->startTimer();
            if (nAi.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional)
            {
                returnErrorWithLog(N_UNEXP_PDU, "Received FF frame with N_TAtype %s", N_TAtypeToString(nAi.N_TAtype));
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

            OSInterfaceLogDebug(tag, "Received FF frame with full message length = %ld", messageLength);

            int64_t availableMemory;
            availableMemoryForRunners->get(&availableMemory);

            if (availableMemoryForRunners->subIfResIsGreaterThanZero(
                    this->messageLength * static_cast<int64_t>(sizeof(uint8_t)))) // Check if there is enough memory
            {
                this->messageData = static_cast<uint8_t*>(osInterface->osMalloc(messageLength * sizeof(uint8_t)));

                if (this->messageData == nullptr)
                {
                    returnErrorWithLog(N_ERROR, "Not enough memory for message length %ld. Available memory is %ld",
                                       messageLength, availableMemory);
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

                updateInternalStatus(SEND_FC);
                result = IN_PROGRESS_FF;
                return result;
            }

            sendFCFrame(OVERFLOW);
            returnErrorWithLog(
                N_ERROR, "Not enough memory for message length %ld. Available memory is %ld. OVERFLOW FC frame sent",
                messageLength, availableMemory);
        }
        default:
            returnErrorWithLog(N_UNEXP_PDU, "Received frame with invalid PDU code %s (%u)",
                               frameCodeToString(frameCode), frameCode);
    }
}

N_Result N_USData_Indication_Runner::runStep_FC_CTS(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is not null");
    }

    timerN_Br->stopTimer();
    OSInterfaceLogVerbose(tag, "Timer N_Br stopped before sending FC frame in %u ms", timerN_Br->getElapsedTime_ms());

    if (sendFCFrame(CONTINUE_TO_SEND) != N_OK)
    {
        returnErrorWithLog(N_ERROR, "Flow control frame could not be sent");
    }

    timerN_Ar->startTimer();
    OSInterfaceLogVerbose(tag, "Timer N_Ar started after sending FC frame");

    result = IN_PROGRESS;
    return result;
}

N_Result N_USData_Indication_Runner::runStep_CF(const CANFrame* receivedFrame)
{
    if (receivedFrame == nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is null");
    }

    if (receivedFrame->identifier.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional)
    {
        returnErrorWithLog(N_UNEXP_PDU, "Received CF frame with N_TAtype %s", N_TAtypeToString(nAi.N_TAtype));
    }

    FrameCode frameCode = static_cast<FrameCode>(receivedFrame->data[0] >> 4);
    if (frameCode != CF_CODE)
    {
        returnErrorWithLog(N_UNEXP_PDU, "Received frame type %s (%u) is not a CF frame", frameCodeToString(frameCode),
                           frameCode);
    }

    uint8_t messageSequenceNumber = (receivedFrame->data[0] & 0b00001111);
    if (messageSequenceNumber != sequenceNumber)
    {
        returnErrorWithLog(N_WRONG_SN, "Received CF frame with wrong sequence number %d. Was expecting %d",
                           messageSequenceNumber, sequenceNumber);
    }

    sequenceNumber++;

    if (receivedFrame->data_length_code <= 1)
    {
        returnErrorWithLog(N_ERROR, "Received CF frame with invalid data length code %d",
                           receivedFrame->data_length_code);
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
        OSInterfaceLogVerbose(tag, "Timer N_Cr stopped after receiving CF frame in %u ms",
                              timerN_Cr->getElapsedTime_ms());
        OSInterfaceLogInfo(tag, "Received message with length %ld (MF)", messageLength);
        result = N_OK;
        updateInternalStatus(MESSAGE_RECEIVED);
    }
    else
    {
        if (effectiveBlockSize == cfReceivedInThisBlock)
        {
            timerN_Cr->stopTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Cr stopped after receiving CF frame in %u ms",
                                  timerN_Cr->getElapsedTime_ms());
            timerN_Br->startTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Br started after receiving CF frame");

            cfReceivedInThisBlock = 0;
            OSInterfaceLogDebug(tag, "CF block size reached.");

            updateInternalStatus(SEND_FC);
        }
        else
        {
            timerN_Cr->startTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Cr started after receiving CF frame");
        }
        result = IN_PROGRESS;
    }
    return result;
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

    OSInterfaceLogDebug(tag, "Sending FC frame with flow status %d, block size %d and STmin %s", fs, effectiveBlockSize,
                        STminToString(stMin));

    if (CanMessageACKQueue->writeFrame(*this, fcFrame))
    {
        updateInternalStatus(AWAITING_FC_ACK);
        return N_OK;
    }

    OSInterfaceLogError(tag, "FC frame could not be sent");
    return N_ERROR;
}

N_Result N_USData_Indication_Runner::checkTimeouts()
{
    uint32_t N_Br_performance = timerN_Br->getElapsedTime_ms() + timerN_Ar->getElapsedTime_ms();
    if (N_Br_performance > N_Br_TIMEOUT_MS)
    {
        OSInterfaceLogWarning(tag, "N_Br performance not met. Elapsed time is %u ms and required is %u",
                              N_Br_performance, N_Br_TIMEOUT_MS);
    }
    if (timerN_Ar->getElapsedTime_ms() > N_Ar_TIMEOUT_MS)
    {
        returnErrorWithLog(N_TIMEOUT_A, "Elapsed time is %u ms and timeout is %u", timerN_Ar->getElapsedTime_ms(),
                           N_Ar_TIMEOUT_MS);
    }
    if (timerN_Cr->getElapsedTime_ms() > N_Cr_TIMEOUT_MS)
    {
        returnErrorWithLog(N_TIMEOUT_Cr, "Elapsed time is %u ms and timeout is %u", timerN_Cr->getElapsedTime_ms(),
                           N_Cr_TIMEOUT_MS);
    }
    return N_OK;
}

uint32_t N_USData_Indication_Runner::getNextTimeoutTime() const
{
    int32_t timeoutAr = timerN_Ar->isTimerRunning()
                            ? (N_Ar_TIMEOUT_MS - static_cast<int32_t>(timerN_Ar->getElapsedTime_ms()))
                            : MAX_TIMEOUT_MS;
    int32_t timeoutCr = timerN_Cr->isTimerRunning()
                            ? (N_Cr_TIMEOUT_MS - static_cast<int32_t>(timerN_Cr->getElapsedTime_ms()))
                            : MAX_TIMEOUT_MS;

    int32_t minTimeout = MIN(timeoutAr, timeoutCr);

    if (minTimeout == timeoutAr)
    {
        OSInterfaceLogVerbose(tag, "Next timeout is N_Ar with %d ms remaining", minTimeout);
    }
    else if (minTimeout == timeoutCr)
    {
        OSInterfaceLogVerbose(tag, "Next timeout is N_Cr with %d ms remaining", minTimeout);
    }

    OSInterfaceLogVerbose(tag, "Next timeout is in %u ms", minTimeout);
    return minTimeout + osInterface->osMillis();
}

uint32_t N_USData_Indication_Runner::getNextRunTime()
{
    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        OSInterfaceLogError(tag, "Failed to acquire mutex");
        result = N_ERROR;
        updateInternalStatus(ERROR);
        return 0;
    }

    uint32_t nextRunTime = getNextTimeoutTime();
    switch (internalStatus)
    {
        case MESSAGE_RECEIVED:
            [[fallthrough]];
        case NOT_RUNNING:
            [[fallthrough]];
        case SEND_FC:
            nextRunTime = 0; // Execute as soon as possible
            OSInterfaceLogDebug(tag, "Next run time is NOW because internalStatus is %s (%d)",
                                internalStatusToString(internalStatus), internalStatus);
            break;
        default:
            OSInterfaceLogDebug(tag, "Next run time is in %ld ms because of next timeout",
                                static_cast<int64_t>(nextRunTime) - osInterface->osMillis());
            break;
    }

    mutex->signal();

    return nextRunTime;
}

void N_USData_Indication_Runner::messageACKReceivedCallback(const CANInterface::ACKResult success)
{
    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        OSInterfaceLogError(tag, "Failed to acquire mutex");
        result = N_ERROR;
        updateInternalStatus(ERROR);
        return;
    }

    OSInterfaceLogDebug(tag, "Running messageACKReceivedCallback with internalStatus = %s (%d) and success = %s",
                        internalStatusToString(internalStatus), internalStatus,
                        CANInterface::ackResultToString(success));

    switch (internalStatus)
    {
        case AWAITING_FC_ACK:
        {
            if (success == CANInterface::ACK_SUCCESS)
            {
                timerN_Ar->stopTimer();
                timerN_Br->clearTimer();
                timerN_Cr->startTimer();
                OSInterfaceLogDebug(tag, "FC ACK received");

                updateInternalStatus(AWAITING_CF);

                if (frameToHoldValid)
                {
                    OSInterfaceLogDebug(tag, "Processing held frame: %s", frameToString(frameToHold));
                    frameToHoldValid = false; // Reset the held frame after processing.
                    runStep_internal(&frameToHold);
                }
            }
            else
            {
                OSInterfaceLogError(tag, "FC ACK failed with result %d", success);
                result = N_ERROR;
                updateInternalStatus(ERROR);
            }
        }
        break;
        default:
            OSInterfaceLogError(tag, "Invalid internalStatus %s (%d)", internalStatusToString(internalStatus),
                                internalStatus);
            result = N_ERROR;
            updateInternalStatus(ERROR);
            break;
    }

    mutex->signal();
}

bool N_USData_Indication_Runner::setBlockSize(const uint8_t blockSize)
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

bool N_USData_Indication_Runner::setSTmin(const STmin stMin)
{
    if (mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        this->stMin = stMin;
        mutex->signal();
        OSInterfaceLogInfo(tag, "STmin set to %s", STminToString(stMin));
        return true;
    }
    return false;
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

const char* N_USData_Indication_Runner::getTAG() const
{
    return this->tag;
}

bool N_USData_Indication_Runner::isThisFrameForMe(const CANFrame& frame) const
{
    bool res = getN_AI().N_AI == frame.identifier.N_AI;
    OSInterfaceLogDebug(tag, "isThisFrameForMe() = %s for frame %s", res ? "true" : "false", frameToString(frame));
    return res;
}

const char* N_USData_Indication_Runner::internalStatusToString(const InternalStatus_t status)
{
    switch (status)
    {
        case NOT_RUNNING:
            return "NOT_RUNNING";
        case SEND_FC:
            return "SEND_FC";
        case AWAITING_FC_ACK:
            return "AWAITING_FC_ACK";
        case AWAITING_CF:
            return "AWAITING_CF";
        case MESSAGE_RECEIVED:
            return "MESSAGE_RECEIVED";
        case ERROR:
            return "ERROR";
        default:
            return "UNKNOWN_STATUS";
    }
}
