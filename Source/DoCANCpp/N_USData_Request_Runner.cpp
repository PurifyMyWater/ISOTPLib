#include "N_USData_Request_Runner.h"
#include <cassert>
#include <cstring>
#include <ctime>

#include "Atomic_int64_t.h"
#include "CANMessageACKQueue.h"

N_USData_Request_Runner::N_USData_Request_Runner(bool& result, const N_AI nAi,
                                                 Atomic_int64_t& availableMemoryForRunners, const Mtype mType,
                                                 const uint8_t* messageData, const uint32_t messageLength,
                                                 OSInterface& osInterface, CANMessageACKQueue& canMessageACKQueue)
{
    result = false;

    this->availableMemoryForRunners = &availableMemoryForRunners;
    this->osInterface               = &osInterface;

    if (this->availableMemoryForRunners->subIfResIsGreaterThanZero(N_USDATA_REQUEST_RUNNER_TAG_SIZE))
    {
        this->tag = static_cast<char*>(this->osInterface->osMalloc(N_USDATA_REQUEST_RUNNER_TAG_SIZE));
        if (this->tag == nullptr)
        {
            return;
        }
        snprintf(this->tag, N_USDATA_REQUEST_RUNNER_TAG_SIZE, "%s%s", N_USDATA_REQUEST_RUNNER_STATIC_TAG,
                 nAiToString(nAi));
    }
    else
    {
        return;
    }

    this->nAi                = nAi;
    this->mType              = Mtype_Unknown;
    this->CanMessageACKQueue = &canMessageACKQueue;
    this->blockSize          = 0;
    this->stMin              = DEFAULT_STMIN;
    this->lastRunTime        = 0;
    this->sequenceNumber = 1; // The first sequence number that is being sent is 1. (0 is reserved for the first frame)

    this->mutex = osInterface.osCreateMutex();
    if (this->mutex == nullptr)
    {
        OSInterfaceLogError(tag, AT "Failed to create mutex");
        return;
    }

    this->internalStatus            = ERROR;
    this->result                    = NOT_STARTED;
    this->messageOffset             = 0;
    this->messageData               = nullptr;
    this->messageLength             = messageLength;
    this->availableMemoryForRunners = &availableMemoryForRunners;
    this->cfSentInThisBlock         = 0;

    this->timerN_As = new Timer_N(osInterface);
    this->timerN_Bs = new Timer_N(osInterface);
    this->timerN_Cs = new Timer_N(osInterface);

    if (this->availableMemoryForRunners->subIfResIsGreaterThanZero(this->messageLength *
                                                                   static_cast<int64_t>(sizeof(uint8_t))) &&
        messageData != nullptr)
    {
        this->messageData = static_cast<uint8_t*>(osInterface.osMalloc(this->messageLength * sizeof(uint8_t)));

        if (this->messageLength == 0)
        {
            this->messageData = static_cast<uint8_t*>(osInterface.osMalloc(1 * sizeof(uint8_t)));
            if (this->messageData != nullptr)
            {
                this->messageData[0] = '\0';
            }
        }
        if (this->messageData == nullptr)
        {
            OSInterfaceLogError(tag, "Not enough memory for message length %u", messageLength);
        }
        else
        {
            memcpy(this->messageData, messageData, this->messageLength);
            this->mType = mType;

            if (this->nAi.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional &&
                this->messageLength > MAX_SF_MESSAGE_LENGTH)
            {
                OSInterfaceLogError(tag, "Message length %u is too long for N_TAtype %d", messageLength,
                                    this->nAi.N_TAtype);
            }
            else
            {
                this->nAi = nAi;

                if (messageLength <= MAX_SF_MESSAGE_LENGTH)
                {
                    OSInterfaceLogDebug(tag, "Message type is Single Frame");
                    internalStatus = NOT_RUNNING_SF;
                }
                else
                {
                    OSInterfaceLogDebug(tag, "Message type is Multiple Frame");
                    internalStatus = NOT_RUNNING_FF;
                }

                result = true;
            }
        }
    }
    else
    {
        OSInterfaceLogError(tag, "Not enough memory for message length %u", messageLength);
    }
}

// Be careful with the destructor. All the pointers used in the destructor need to be initialized to nullptr. Otherwise,
// the destructor may attempt a free on an invalid pointer.
N_USData_Request_Runner::~N_USData_Request_Runner()
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
        availableMemoryForRunners->add(N_USDATA_REQUEST_RUNNER_TAG_SIZE);
    }

    delete timerN_As;
    delete timerN_Bs;
    delete timerN_Cs;
    delete mutex;
}

N_Result N_USData_Request_Runner::sendCFFrame()
{
    CANFrame cfFrame   = NewCANFrameDoCANCpp();
    cfFrame.identifier = nAi;

    int64_t remainingBytes  = messageLength - messageOffset;
    uint8_t frameDataLength = remainingBytes > MAX_CF_MESSAGE_LENGTH ? MAX_CF_MESSAGE_LENGTH : remainingBytes;

    cfFrame.data[0] = (CF_CODE << 4) | (sequenceNumber & 0b00001111);       // (0b0010xxxx) | SN (0bxxxxllll)
    memcpy(&cfFrame.data[1], &messageData[messageOffset], frameDataLength); // Payload data
    messageOffset += frameDataLength;

    cfFrame.data_length_code = frameDataLength + 1; // 1 byte for N_PCI_SF

    OSInterfaceLogDebug(tag, "Sending CF #%d in block with %d data bytes", cfSentInThisBlock + 1, frameDataLength);

    if (CanMessageACKQueue->writeFrame(*this, cfFrame))
    {
        cfSentInThisBlock++;
        sequenceNumber++;
        timerN_As->startTimer();
        OSInterfaceLogVerbose(tag, "Timer N_As started after sending CF");

        updateInternalStatus(AWAITING_CF_ACK);
        result = IN_PROGRESS;
        return result;
    }

    OSInterfaceLogError(tag, "CF frame could not be sent");
    result = N_ERROR;
    return result;
}

N_Result N_USData_Request_Runner::checkTimeouts()
{
    uint32_t N_Cs_performance = timerN_Cs->getElapsedTime_ms() + timerN_As->getElapsedTime_ms();
    if (N_Cs_performance > N_Cs_TIMEOUT_MS)
    {
        OSInterfaceLogWarning(tag, "N_Cs performance not met. Elapsed time is %u ms and required is %u",
                              N_Cs_performance, N_Cs_TIMEOUT_MS);
    }
    if (timerN_As->getElapsedTime_ms() > N_As_TIMEOUT_MS)
    {
        returnErrorWithLog(N_TIMEOUT_A, "Elapsed time is %u ms and timeout is %u", timerN_As->getElapsedTime_ms(),
                           N_As_TIMEOUT_MS);
    }
    if (timerN_Bs->getElapsedTime_ms() > N_Bs_TIMEOUT_MS)
    {
        returnErrorWithLog(N_TIMEOUT_Bs, "Elapsed time is %u ms and timeout is %u", timerN_Bs->getElapsedTime_ms(),
                           N_Bs_TIMEOUT_MS);
    }
    return N_OK;
}

N_Result N_USData_Request_Runner::runStep(CANFrame* receivedFrame)
{
    OSInterfaceLogVerbose(tag, "Running step with internalStatus = %s (%d) and frame %s",
                          internalStatusToString(internalStatus), internalStatus,
                          receivedFrame != nullptr ? frameToString(*receivedFrame) : "null");

    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        returnErrorWithLog(N_ERROR, "Failed to acquire mutex");
    }

    N_Result res = checkTimeouts();

    if (res != N_OK)
    {
        mutex->signal();
        OSInterfaceLogError(tag, "Timeout occurred: %s", N_ResultToString(res));
        return res;
    }

    switch (internalStatus)
    {
        case NOT_RUNNING_SF:
            res = runStep_SF(receivedFrame);
            break;
        case NOT_RUNNING_FF:
            res = runStep_FF(receivedFrame);
            break;
        case AWAITING_FirstFC:                     // We got the message or timeout.
            res = runStep_FC(receivedFrame, true); // First FC
            break;
        case SEND_CF:
            res = runStep_CF(receivedFrame);
            break;
        case AWAITING_FC:
            res = runStep_FC(receivedFrame);
            break;
        case MESSAGE_SENT:
            result = N_OK; // If the message is successfully sent, return N_OK to allow DoCanCpp to call the callback.
            res    = result;
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

    mutex->signal();
    return res;
}

N_Result N_USData_Request_Runner::runStep_CF(const CANFrame* receivedFrame)
{
    timerN_Cs->stopTimer();
    OSInterfaceLogVerbose(tag, "Timer N_Cs stopped before sending CF in %u ms", timerN_Cs->getElapsedTime_ms());
    if (receivedFrame != nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is not null");
    }

    result = sendCFFrame();
    return result;
}

N_Result N_USData_Request_Runner::runStep_FF(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is not null");
    }

    CANFrame ffFrame   = NewCANFrameDoCANCpp();
    ffFrame.identifier = nAi;

    if (messageLength < MIN_FF_DL_WITH_ESCAPE_SEQUENCE)
    {
        ffFrame.data[0] = FF_CODE << 4 | messageLength >> 8; // N_PCI_FF (0b0001xxxx) | messageLength (0bxxxxllll)
        ffFrame.data[1] = messageLength & 0b11111111;        // messageLength LSB

        memcpy(&ffFrame.data[2], messageData, 6); // Payload data
        messageOffset = 6;
    }
    else
    {
        ffFrame.data[0] = FF_CODE << 4; // N_PCI_FF (0b00010000)
        ffFrame.data[1] = 0;
        *reinterpret_cast<uint32_t*>(&ffFrame.data[2]) =
            static_cast<uint32_t>(messageLength); // copy messageLength (4 bytes) in the bytes #2 to #5.

        ffFrame.data[2] = messageLength >> 24 & 0b11111111;
        ffFrame.data[3] = messageLength >> 16 & 0b11111111;
        ffFrame.data[4] = messageLength >> 8 & 0b11111111;
        ffFrame.data[5] = messageLength & 0b11111111;

        memcpy(&ffFrame.data[6], messageData, 2); // Payload data
        messageOffset = 2;
    }

    OSInterfaceLogDebug(tag, "Sending FF frame with data length %u", messageOffset);

    ffFrame.data_length_code = CAN_FRAME_MAX_DLC;

    if (CanMessageACKQueue->writeFrame(*this, ffFrame))
    {
        timerN_As->startTimer();
        OSInterfaceLogVerbose(tag, "Timer N_As started after sending FF frame");

        updateInternalStatus(AWAITING_FF_ACK);
        result = IN_PROGRESS;
        return result;
    }

    returnErrorWithLog(N_ERROR, "FF frame could not be sent");
}

N_Result N_USData_Request_Runner::runStep_SF(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        returnErrorWithLog(N_ERROR, "received frame is not null");
    }

    timerN_As->startTimer();
    OSInterfaceLogVerbose(tag, "Timer N_As started before sending SF frame");

    CANFrame sfFrame   = NewCANFrameDoCANCpp();
    sfFrame.identifier = nAi;

    sfFrame.data[0] = messageLength;                      // N_PCI_SF (0b0000xxxx) | messageLength (0bxxxxllll)
    memcpy(&sfFrame.data[1], messageData, messageLength); // Payload data

    sfFrame.data_length_code = messageLength + 1; // 1 byte for N_PCI_SF

    if (CanMessageACKQueue->writeFrame(*this, sfFrame))
    {
        OSInterfaceLogDebug(tag, "Sending SF frame with data length %ld", messageLength);
        updateInternalStatus(AWAITING_SF_ACK);
        result = IN_PROGRESS;
        return result;
    }
    OSInterfaceLogError(tag, "SF frame could not be sent");
    result = N_ERROR;
    return result;
}

N_Result N_USData_Request_Runner::runStep_FC(const CANFrame* receivedFrame, const bool firstFC)
{
    FlowStatus fs;
    uint8_t    bs;
    STmin      stM;
    if (parseFCFrame(receivedFrame, fs, bs, stM) != N_OK)
    {
        return result; // All the other error parameters are already set.
    }

    switch (fs)
    {
        case CONTINUE_TO_SEND:
        {
            OSInterfaceLogDebug(tag, "Received FC frame with flow status CONTINUE_TO_SEND");
            blockSize         = bs;
            cfSentInThisBlock = 0;
            stMin             = stM;

            timerN_Bs->stopTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Bs stopped after receiving FC frame in %u ms",
                                  timerN_Bs->getElapsedTime_ms());
            timerN_Cs->startTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Cs started after receiving FC frame");

            result = IN_PROGRESS;
            updateInternalStatus(SEND_CF);
            return result;
        }
        case WAIT:
        {
            OSInterfaceLogDebug(tag, "Received FC frame with flow status WAIT");
            // Restart N_Bs timer
            timerN_Bs->startTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Bs started after receiving FC frame");
            updateInternalStatus(AWAITING_FC);
            result = IN_PROGRESS;
            return result;
        }
        case OVERFLOW:
            OSInterfaceLogDebug(tag, "Received FC frame with flow status OVERFLOW");
            if (firstFC)
            {
                returnError(N_BUFFER_OVFLW);
            }
            [[fallthrough]]; // If an overflow is received in any other moment than the first FC, is an invalid flow
                             // status.
        default:
            returnErrorWithLog(N_INVALID_FS, "Received FC frame with invalid flow status %d", fs);
    }
}

bool N_USData_Request_Runner::awaitingMessage() const
{
    return internalStatus == AWAITING_FC || internalStatus == AWAITING_FirstFC;
}

uint32_t N_USData_Request_Runner::getNextTimeoutTime() const
{
    int32_t timeoutAs = timerN_As->isTimerRunning()
                            ? (N_As_TIMEOUT_MS - static_cast<int32_t>(timerN_As->getElapsedTime_ms()))
                            : MAX_TIMEOUT_MS;
    int32_t timeoutBs = timerN_Bs->isTimerRunning()
                            ? (N_Bs_TIMEOUT_MS - static_cast<int32_t>(timerN_Bs->getElapsedTime_ms()))
                            : MAX_TIMEOUT_MS;
    int32_t timeoutCs =
        timerN_Cs->isTimerRunning()
            ? (static_cast<int32_t>(getStMinInMs(stMin)) - static_cast<int32_t>(timerN_Cs->getElapsedTime_ms()))
            : MAX_TIMEOUT_MS;

    int32_t minTimeoutAsBs = MIN(timeoutAs, timeoutBs);
    int32_t minTimeout     = MIN(minTimeoutAsBs, timeoutCs);

    if (minTimeout == timeoutAs)
    {
        OSInterfaceLogVerbose(tag, "Next timeout is N_As with %d ms remaining", minTimeout);
    }
    else if (minTimeout == timeoutBs)
    {
        OSInterfaceLogVerbose(tag, "Next timeout is N_Bs with %d ms remaining", minTimeout);
    }
    else if (minTimeout == timeoutCs)
    {
        OSInterfaceLogVerbose(tag, "Next timeout is N_Cs with %d ms remaining", minTimeout);
    }

    return minTimeout + osInterface->osMillis();
}

uint32_t N_USData_Request_Runner::getNextRunTime() const
{
    uint32_t nextRunTime = getNextTimeoutTime();
    switch (internalStatus)
    {
        case MESSAGE_SENT:
            [[fallthrough]];
        case NOT_RUNNING_SF:
            [[fallthrough]];
        case NOT_RUNNING_FF:
            nextRunTime = 0; // Execute as soon as possible
            OSInterfaceLogDebug(tag, "Next run time is in %u ms because internalStatus is %s", nextRunTime,
                                internalStatusToString(internalStatus));
            break;
        default:
            OSInterfaceLogDebug(tag, "Next run time is in %u ms because of next timeout",
                                nextRunTime - osInterface->osMillis());
            break;
    }
    return nextRunTime;
}

void N_USData_Request_Runner::messageACKReceivedCallback(const CANInterface::ACKResult success)
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
        case AWAITING_SF_ACK:
        {
            OSInterfaceLogDebug(tag, "Received SF ACK");
            if (success == CANInterface::ACK_SUCCESS)
            {
                timerN_As->stopTimer();
                timerN_Cs->clearTimer();
                OSInterfaceLogVerbose(tag, "Timer N_As stopped after receiving SF ACK in %u ms",
                                      timerN_As->getElapsedTime_ms());
                updateInternalStatus(MESSAGE_SENT);
            }
            else
            {
                OSInterfaceLogError(tag, "SF ACK failed with result %d", success);
                result = N_ERROR;
                updateInternalStatus(ERROR);
            }
            break;
        }
        case AWAITING_FF_ACK:
        {
            OSInterfaceLogDebug(tag, "Received FF ACK");
            if (success == CANInterface::ACK_SUCCESS)
            {
                updateInternalStatus(AWAITING_FirstFC);
                timerN_As->stopTimer();
                timerN_Cs->clearTimer();
                OSInterfaceLogVerbose(tag, "Timer N_As stopped after receiving FF ACK in %u ms",
                                      timerN_As->getElapsedTime_ms());
                timerN_Bs->startTimer();
                OSInterfaceLogVerbose(tag, "Timer N_Bs started after receiving FF ACK");
            }
            else
            {
                OSInterfaceLogError(tag, "FF ACK failed with result %d", success);
                result = N_ERROR;
                updateInternalStatus(ERROR);
            }
            break;
        }
        case AWAITING_CF_ACK:
        {
            OSInterfaceLogDebug(tag, "Received CF ACK");
            if (success == CANInterface::ACK_SUCCESS)
            {
                timerN_As->stopTimer();
                timerN_Cs->clearTimer();
                OSInterfaceLogVerbose(tag, "Timer N_As stopped after receiving CF ACK in %u ms",
                                      timerN_As->getElapsedTime_ms());

                if (messageOffset == messageLength)
                {
                    timerN_As->stopTimer();
                    timerN_Cs->clearTimer();
                    OSInterfaceLogVerbose(tag, "Timer N_As stopped after receiving CF ACK in %u ms",
                                          timerN_As->getElapsedTime_ms());
                    updateInternalStatus(MESSAGE_SENT);
                }
                else if (cfSentInThisBlock == blockSize)
                {
                    updateInternalStatus(AWAITING_FC);
                    timerN_Bs->startTimer();
                    OSInterfaceLogVerbose(tag, "Timer N_Bs started after receiving CF ACK");
                }
                else
                {
                    timerN_Cs->startTimer();
                    OSInterfaceLogVerbose(tag, "Timer N_Cs started after receiving CF ACK");
                    updateInternalStatus(SEND_CF);
                }
            }
            else
            {
                OSInterfaceLogError(tag, "CF ACK failed with result %d", success);
                result = N_ERROR;
                updateInternalStatus(ERROR);
            }
            break;
        }
        default:
            OSInterfaceLogError(tag, "Invalid internalStatus %s (%d)", internalStatusToString(internalStatus),
                                internalStatus);
            result = N_ERROR;
            updateInternalStatus(ERROR);
            break;
    }

    mutex->signal();
}

N_Result N_USData_Request_Runner::parseFCFrame(const CANFrame* receivedFrame, FlowStatus& fs, uint8_t& blcksize,
                                               STmin& stM)
{
    if (receivedFrame == nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is null");
    }

    timerN_Bs->stopTimer();
    OSInterfaceLogVerbose(tag, "Timer N_Bs stopped after receiving FC frame in %u ms", timerN_Bs->getElapsedTime_ms());
    timerN_Cs->startTimer();
    OSInterfaceLogVerbose(tag, "Timer N_Cs started after receiving FC frame");

    if (receivedFrame->identifier.N_TAtype != N_TATYPE_5_CAN_CLASSIC_29bit_Physical)
    {
        returnErrorWithLog(N_ERROR, "Received frame is of type %s. Expected N_TATYPE_5_CAN_CLASSIC_29bit_Physical",
                           N_TAtypeToString(receivedFrame->identifier.N_TAtype));
    }

    if (receivedFrame->data_length_code != FC_MESSAGE_LENGTH)
    {
        returnErrorWithLog(N_ERROR, "Received frame has invalid data length code %d", receivedFrame->data_length_code);
    }

    if ((receivedFrame->data[0] >> 4 & 0b00001111) != FC_CODE)
    {
        returnErrorWithLog(N_ERROR, "Received frame is not a FC frame");
    }

    fs = static_cast<FlowStatus>(receivedFrame->data[0] & 0b00001111);
    if (fs >= INVALID_FS)
    {
        returnErrorWithLog(N_ERROR, "Received frame has invalid flow status %d", fs);
    }

    blcksize = receivedFrame->data[1];

    if (receivedFrame->data[2] <= 0x7F)
    {
        stM.unit  = ms;
        stM.value = receivedFrame->data[2];
    }
    else if (receivedFrame->data[2] >= 0xF1 && receivedFrame->data[2] <= 0xF9)
    {
        stM.unit  = usX100;
        stM.value = receivedFrame->data[2] & 0x0F;
    }
    else // Reserved values -> max stMin value
    {
        OSInterfaceLogWarning(tag, "FC frame has reserved STmin value %d. Defaulting to 127", receivedFrame->data[2]);
        stM.unit  = ms;
        stM.value = 127;
    }

    return N_OK;
}

N_AI N_USData_Request_Runner::getN_AI() const
{
    return nAi;
}

uint8_t* N_USData_Request_Runner::getMessageData() const
{
    return messageData;
}

uint32_t N_USData_Request_Runner::getMessageLength() const
{
    return messageLength;
}

N_Result N_USData_Request_Runner::getResult() const
{
    return result;
}

Mtype N_USData_Request_Runner::getMtype() const
{
    return mType;
}

N_USData_Request_Runner::RunnerType N_USData_Request_Runner::getRunnerType() const
{
    return RunnerRequestType;
}

const char* N_USData_Request_Runner::getTAG() const
{
    return this->tag;
}

bool N_USData_Request_Runner::isThisFrameForMe(const CANFrame& frame) const
{
    N_AI runnerN_AI = getN_AI();
    N_AI frameN_AI  = frame.identifier;

    bool res = runnerN_AI.N_NFA_Header == frameN_AI.N_NFA_Header;
    res &= runnerN_AI.N_NFA_Padding == frameN_AI.N_NFA_Padding;
    res &= runnerN_AI.N_TAtype == frameN_AI.N_TAtype;
    res &= runnerN_AI.N_TA == frameN_AI.N_SA;
    res &= runnerN_AI.N_SA == frameN_AI.N_TA;

    return res;
}

const char* N_USData_Request_Runner::internalStatusToString(const InternalStatus_t status)
{
    switch (status)
    {
        case NOT_RUNNING_SF:
            return "NOT_RUNNING_SF";
        case AWAITING_SF_ACK:
            return "AWAITING_SF_ACK";
        case NOT_RUNNING_FF:
            return "NOT_RUNNING_FF";
        case AWAITING_FF_ACK:
            return "AWAITING_FF_ACK";
        case AWAITING_FirstFC:
            return "AWAITING_FirstFC";
        case AWAITING_FC:
            return "AWAITING_FC";
        case SEND_CF:
            return "SEND_CF";
        case AWAITING_CF_ACK:
            return "AWAITING_CF_ACK";
        case MESSAGE_SENT:
            return "MESSAGE_SENT";
        case ERROR:
            return "ERROR";
        default:
            return "UNKNOWN_STATUS";
    }
}
