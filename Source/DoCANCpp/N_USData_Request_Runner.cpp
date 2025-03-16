#include "N_USData_Request_Runner.h"

#include <cassert>

#include "Atomic_int64_t.h"
#include "CANMessageACKQueue.h"

#include <cstring>

N_USData_Request_Runner::N_USData_Request_Runner(bool* result, N_AI nAi, Atomic_int64_t& availableMemoryForRunners,
                                                 Mtype mType, const uint8_t* messageData, uint32_t messageLength,
                                                 OSInterface& osInterface, CANMessageACKQueue& canMessageACKQueue)
{
    this->TAG = "DoCANCpp_RequestRunner";

    this->nAi                = nAi;
    this->mType              = Mtype_Unknown;
    this->osInterface        = &osInterface;
    this->CanMessageACKQueue = &canMessageACKQueue;
    this->blockSize          = 0;
    this->stMin              = {0, ms};
    this->lastRunTime        = 0;
    this->sequenceNumber = 1; // The first sequence number that is being sent is 1. (0 is reserved for the first frame)

    this->mutex = osInterface.osCreateMutex();
    assert(this->mutex != nullptr && "Failed to create mutex");

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

        if (this->messageData == nullptr)
        {
            OSInterfaceLogError(tag, "Not enough memory for message length %u", messageLength);
            *result = false;
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
                *result = false;
            }
            else
            {
                this->nAi = nAi;

                if (messageLength <= MAX_SF_MESSAGE_LENGTH)
                {
                    OSInterfaceLogDebug(tag, "Message type is Single Frame");
                    this->internalStatus = NOT_RUNNING_SF;
                }
                else
                {
                    OSInterfaceLogDebug(tag, "Message type is First Frame");
                    this->internalStatus = NOT_RUNNING_FF;
                }

                *result = true;
            }
        }
    }
    else
    {
        OSInterfaceLogError(tag, "Not enough memory for message length %u", messageLength);
        *result = false;
    }
}

N_USData_Request_Runner::~N_USData_Request_Runner()
{
    OSInterfaceLogDebug(tag, "Deleting runner");

    if (this->messageData == nullptr)
    {
        return;
    }

    osInterface->osFree(this->messageData);
    availableMemoryForRunners->add(messageLength * static_cast<int64_t>(sizeof(uint8_t)));

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

    OSInterfaceLogDebug(tag, "Sending CF #%d in block with %d data bytes", cfSentInThisBlock, frameDataLength);

    if (CanMessageACKQueue->writeFrame(*this, cfFrame))
    {
        cfSentInThisBlock++;
        sequenceNumber++;
        timerN_As->startTimer();

        internalStatus = AWAITING_CF_ACK;
        result         = IN_PROGRESS;
        return result;
    }

    OSInterfaceLogError(tag, "CF frame could not be sent");
    result = N_ERROR;
    return result;
}

N_Result N_USData_Request_Runner::checkTimeouts()
{
#if !DOCANCPP_DISABLE_TIMEOUTS
    if (timerN_As->getElapsedTime_ms() > N_As_TIMEOUT_MS)
    {
        returnErrorWithLog(N_TIMEOUT_A, "Elapsed time is %u ms", timerN_As->getElapsedTime_ms());
    }
    if (timerN_Bs->getElapsedTime_ms() > N_Bs_TIMEOUT_MS)
    {
        returnErrorWithLog(N_TIMEOUT_Bs, "Elapsed time is %u ms", timerN_Bs->getElapsedTime_ms());
        returnError(N_TIMEOUT_Bs);
    }
#endif
    return N_OK;
}

N_Result N_USData_Request_Runner::run_step(CANFrame* receivedFrame)
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
        case NOT_RUNNING_SF:
            res = run_step_SF(receivedFrame);
            break;
        case NOT_RUNNING_FF:
            res = run_step_FF(receivedFrame);
            break;
        case AWAITING_FirstFC:                      // We got the message or timeout.
            res = run_step_FC(receivedFrame, true); // First FC
            break;
        case SEND_CF:
            res = run_step_CF(receivedFrame);
            break;
        case AWAITING_FC:
            res = run_step_FC(receivedFrame);
            break;
        case MESSAGE_SENT:
            result = N_OK; // If the message is successfully sent, return N_OK to allow DoCanCpp to call the callback.
            res    = result;
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

N_Result N_USData_Request_Runner::run_step_CF(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        returnErrorWithLog(N_ERROR, "Received frame is not null");
    }

    result = sendCFFrame();
    return result;
}

N_Result N_USData_Request_Runner::run_step_FF(const CANFrame* receivedFrame)
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
        OSInterfaceLogVerbose(tag, "Timer N_As started after sending FF frame in %u ms",
                              timerN_As->getElapsedTime_ms());

        internalStatus = AWAITING_FF_ACK;
        result         = IN_PROGRESS;
        return result;
    }

    returnErrorWithLog(N_ERROR, "FF frame could not be sent");
}

N_Result N_USData_Request_Runner::run_step_SF(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        returnErrorWithLog(N_ERROR, "received frame is not null");
    }

    timerN_As->startTimer();
    OSInterfaceLogVerbose(tag, "Timer N_As started after sending SF frame in %u ms", timerN_As->getElapsedTime_ms());

    CANFrame sfFrame   = NewCANFrameDoCANCpp();
    sfFrame.identifier = nAi;

    sfFrame.data[0] = messageLength;                      // N_PCI_SF (0b0000xxxx) | messageLength (0bxxxxllll)
    memcpy(&sfFrame.data[1], messageData, messageLength); // Payload data

    sfFrame.data_length_code = messageLength + 1; // 1 byte for N_PCI_SF

    if (CanMessageACKQueue->writeFrame(*this, sfFrame))
    {
        OSInterfaceLogDebug(tag, "Sending SF frame with data length %ld", messageLength);
        internalStatus = AWAITING_SF_ACK;
        result         = IN_PROGRESS;
        return result;
    }
    OSInterfaceLogError(tag, "SF frame could not be sent");
    result = N_ERROR;
    return result;
}

N_Result N_USData_Request_Runner::run_step_FC(const CANFrame* receivedFrame, const bool firstFC)
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
            OSInterfaceLogVerbose(tag, "Timer N_Cs started after receiving FC frame in %u ms",
                                  timerN_Cs->getElapsedTime_ms());

            result         = IN_PROGRESS;
            internalStatus = SEND_CF;
            return result;
        }
        case WAIT:
        {
            OSInterfaceLogDebug(tag, "Received FC frame with flow status WAIT");
            // Restart N_Bs timer
            timerN_Bs->startTimer();
            OSInterfaceLogVerbose(tag, "Timer N_Bs started after receiving FC frame in %u ms",
                                  timerN_Bs->getElapsedTime_ms());
            internalStatus = AWAITING_FC;
            result         = IN_PROGRESS;
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
    uint32_t timeoutAs      = timerN_As->getStartTimeStamp() + N_As_TIMEOUT_MS;
    uint32_t timeoutBs      = timerN_Bs->getStartTimeStamp() + N_Bs_TIMEOUT_MS;
    uint32_t timeoutCs      = timerN_Cs->getStartTimeStamp() + getStMinInMs(stMin);
    uint32_t minTimeoutAsBs = MIN(timeoutAs, timeoutBs);
    uint32_t minTimeout     = MIN(minTimeoutAsBs, timeoutCs);

    OSInterfaceLogVerbose(tag, "Next timeout is in %u ms", minTimeout - osInterface->osMillis());
    return minTimeout;
}

uint32_t N_USData_Request_Runner::getNextRunTime() const
{
    uint32_t nextRunTime = getNextTimeoutTime();
    switch (internalStatus)
    {
        case MESSAGE_SENT:
            [[fallthrough]]; // Be careful with the fallthrough, we want to execute ASAP.
        case NOT_RUNNING_SF:
            [[fallthrough]];
        case NOT_RUNNING_FF:
            nextRunTime = 0; // Execute as soon as possible
            break;
        default:
            break;
    }

    OSInterfaceLogDebug(tag, "Next run time is in %u ms", nextRunTime - osInterface->osMillis());
    return nextRunTime;
}

void N_USData_Request_Runner::messageACKReceivedCallback(CANInterface::ACKResult success)
{
    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        OSInterfaceLogError(tag, "Failed to acquire mutex");
        result         = N_ERROR;
        internalStatus = ERROR;
        return;
    }

    switch (internalStatus)
    {
        case AWAITING_SF_ACK:
        {
            OSInterfaceLogDebug(tag, "Received SF ACK");
            if (success == CANInterface::ACK_SUCCESS)
            {
                timerN_As->stopTimer();
                OSInterfaceLogVerbose(tag, "Timer N_As stopped after receiving SF ACK in %u ms",
                                      timerN_As->getElapsedTime_ms());
                internalStatus = MESSAGE_SENT;
            }
            else
            {
                OSInterfaceLogError(tag, "SF ACK failed with result %d", success);
                result         = N_ERROR;
                internalStatus = ERROR;
            }
            break;
        }
        case AWAITING_FF_ACK:
        {
            OSInterfaceLogDebug(tag, "Received FF ACK");
            if (success == CANInterface::ACK_SUCCESS)
            {
                internalStatus = AWAITING_FirstFC;
                timerN_As->stopTimer();
                OSInterfaceLogVerbose(tag, "Timer N_As stopped after receiving FF ACK in %u ms",
                                      timerN_As->getElapsedTime_ms());
                timerN_Bs->startTimer();
                OSInterfaceLogVerbose(tag, "Timer N_Bs started after receiving FF ACK in %u ms",
                                      timerN_Bs->getElapsedTime_ms());
            }
            else
            {
                OSInterfaceLogError(tag, "FF ACK failed with result %d", success);
                result         = N_ERROR;
                internalStatus = ERROR;
            }
            break;
        }
        case AWAITING_CF_ACK:
        {
            OSInterfaceLogDebug(tag, "Received CF ACK");
            if (success == CANInterface::ACK_SUCCESS)
            {
                timerN_As->stopTimer();
                OSInterfaceLogVerbose(tag, "Timer N_As stopped after receiving CF ACK in %u ms",
                                      timerN_As->getElapsedTime_ms());

                if (messageOffset == messageLength)
                {
                    timerN_As->stopTimer();
                    OSInterfaceLogVerbose(tag, "Timer N_As stopped after receiving CF ACK in %u ms",
                                          timerN_As->getElapsedTime_ms());
                    internalStatus = MESSAGE_SENT;
                }
                else if (cfSentInThisBlock == blockSize)
                {
                    internalStatus = AWAITING_FC;
                    timerN_Bs->startTimer();
                    OSInterfaceLogVerbose(tag, "Timer N_Bs started after receiving CF ACK in %u ms",
                                          timerN_Bs->getElapsedTime_ms());
                }
                else
                {
                    timerN_Cs->startTimer();
                    OSInterfaceLogVerbose(tag, "Timer N_Cs started after receiving CF ACK in %u ms",
                                          timerN_Cs->getElapsedTime_ms());
                    internalStatus = SEND_CF;
                }
            }
            else
            {
                OSInterfaceLogError(tag, "CF ACK failed with result %d", success);
                result         = N_ERROR;
                internalStatus = ERROR;
            }
            break;
        }
        default:
            OSInterfaceLogError(tag, "Invalid internal status %d", internalStatus);
            result         = N_ERROR;
            internalStatus = ERROR;
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
    OSInterfaceLogVerbose(tag, "Timer N_Cs started after receiving FC frame in %u ms", timerN_Cs->getElapsedTime_ms());

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
