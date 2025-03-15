#include "N_USData_Request_Runner.h"
#include <cassert>
#include <cstring>
#include "Atomic_int64_t.h"
#include "CANMessageACKQueue.h"

N_USData_Request_Runner::N_USData_Request_Runner(bool& result, N_AI nAi, Atomic_int64_t& availableMemoryForRunners, Mtype mType, const uint8_t* messageData, uint32_t messageLength,
                                                 OSInterface& osInterface, CANMessageACKQueue& canMessageACKQueue)
{
    result = false;

    this->availableMemoryForRunners = &availableMemoryForRunners;
    this->osInterface = &osInterface;

    if (this->availableMemoryForRunners->subIfResIsGreaterThanZero(N_USDATA_REQUEST_RUNNER_TAG_SIZE))
    {
        this->tag = static_cast<char*>(this->osInterface->osMalloc(N_USDATA_REQUEST_RUNNER_TAG_SIZE));
        if (this->tag == nullptr)
        {
            return;
        }
        snprintf(this->tag, N_USDATA_REQUEST_RUNNER_TAG_SIZE, "%s%s", N_USDATA_REQUEST_RUNNER_STATIC_TAG, nAiToString(nAi));
    }
    else
    {
        return;
    }

    this->nAi = nAi;
    this->mType = Mtype_Unknown;
    this->CanMessageACKQueue = &canMessageACKQueue;
    this->blockSize = 0;
    this->stMin = {0, ms};
    this->lastRunTime = 0;
    this->sequenceNumber = 1; // The first sequence number that is being sent is 1. (0 is reserved for the first frame)

    this->mutex = osInterface.osCreateMutex();
    assert(this->mutex != nullptr && "Failed to create mutex");

    this->internalStatus = ERROR;
    this->result = NOT_STARTED;
    this->messageOffset = 0;
    this->runnerType = RunnerRequestType;
    this->messageData = nullptr;
    this->messageLength = messageLength;
    this->cfSentInThisBlock = 0;

    this->timerN_As = new Timer_N(osInterface);
    this->timerN_Bs = new Timer_N(osInterface);
    this->timerN_Cs = new Timer_N(osInterface);

    if (this->availableMemoryForRunners->subIfResIsGreaterThanZero(this->messageLength * static_cast<int64_t>(sizeof(uint8_t))) && messageData != nullptr)
    {
        this->messageData = static_cast<uint8_t*>(osInterface.osMalloc(this->messageLength * sizeof(uint8_t)));

        if (this->messageData != nullptr)
        {
            memcpy(this->messageData, messageData, this->messageLength);
            this->mType = mType;

            if (!(this->nAi.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional && this->messageLength > MAX_SF_MESSAGE_LENGTH))
            {
                this->nAi = nAi;

                if (messageLength <= MAX_SF_MESSAGE_LENGTH)
                {
                    this->internalStatus = NOT_RUNNING_SF;
                }
                else
                {
                    this->internalStatus = NOT_RUNNING_FF;
                }

                result = true;
            }
        }
    }
}

N_USData_Request_Runner::~N_USData_Request_Runner()
{
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
    CANFrame cfFrame = NewCANFrameDoCANCpp();
    cfFrame.identifier = nAi;

    int64_t remainingBytes = messageLength - messageOffset;
    uint8_t frameDataLength = remainingBytes > MAX_CF_MESSAGE_LENGTH ? MAX_CF_MESSAGE_LENGTH : remainingBytes;

    cfFrame.data[0] = (CF_CODE << 4) | (sequenceNumber & 0b00001111); // (0b0010xxxx) | SN (0bxxxxllll)
    memcpy(&cfFrame.data[1], &messageData[messageOffset], frameDataLength); // Payload data
    messageOffset += frameDataLength;

    cfFrame.data_length_code = frameDataLength + 1; // 1 byte for N_PCI_SF

    if (CanMessageACKQueue->writeFrame(*this, cfFrame))
    {
        cfSentInThisBlock++;
        sequenceNumber++;
        timerN_As->startTimer();

        internalStatus = AWAITING_CF_ACK;
        result = IN_PROGRESS;
        return result;
    }
    result = N_ERROR;
    return result;
}

N_Result N_USData_Request_Runner::checkTimeouts()
{
#if !DOCANCPP_DISABLE_TIMEOUTS
    if (timerN_As->getElapsedTime_ms() > N_As_TIMEOUT_MS)
    {
        returnError(N_TIMEOUT_A);
    }
    if (timerN_Bs->getElapsedTime_ms() > N_Bs_TIMEOUT_MS)
    {
        returnError(N_TIMEOUT_Bs);
    }
#endif
    return N_OK;
}

N_Result N_USData_Request_Runner::run_step(CANFrame* receivedFrame)
{
    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        result = N_ERROR;
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
        case NOT_RUNNING_SF:
            res = run_step_SF(receivedFrame);
            break;
        case NOT_RUNNING_FF:
            res = run_step_FF(receivedFrame);
            break;
        case AWAITING_FirstFC: // We got the message or timeout.
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
            res = result;
            break;
        case ERROR:
            res = result;
            break;
        default:
            assert(false && "Invalid internal status: Maybe DOCANCPP_DISABLE_TIMEOUTS is true?");
    }

    lastRunTime = osInterface->osMillis();

    mutex->signal();
    return res;
}

N_Result N_USData_Request_Runner::run_step_CF(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        returnError(N_ERROR);
    }

    result = sendCFFrame();
    return result;
}

N_Result N_USData_Request_Runner::run_step_FF(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        returnError(N_ERROR);
    }

    CANFrame ffFrame = NewCANFrameDoCANCpp();
    ffFrame.identifier = nAi;


    if (messageLength < MIN_FF_DL_WITH_ESCAPE_SEQUENCE)
    {
        ffFrame.data[0] = FF_CODE << 4 | messageLength >> 8; // N_PCI_FF (0b0001xxxx) | messageLength (0bxxxxllll)
        ffFrame.data[1] = messageLength & 0b11111111; // messageLength LSB

        memcpy(&ffFrame.data[2], messageData, 6); // Payload data
        messageOffset = 6;
    }
    else
    {
        ffFrame.data[0] = FF_CODE << 4; // N_PCI_FF (0b00010000)
        ffFrame.data[1] = 0;
        *reinterpret_cast<uint32_t*>(&ffFrame.data[2]) = static_cast<uint32_t>(messageLength); // copy messageLength (4 bytes) in the bytes #2 to #5.

        ffFrame.data[2] = messageLength >> 24 & 0b11111111;
        ffFrame.data[3] = messageLength >> 16 & 0b11111111;
        ffFrame.data[4] = messageLength >> 8 & 0b11111111;
        ffFrame.data[5] = messageLength & 0b11111111;

        memcpy(&ffFrame.data[6], messageData, 2); // Payload data
        messageOffset = 2;
    }

    ffFrame.data_length_code = CAN_FRAME_MAX_DLC;

    if (CanMessageACKQueue->writeFrame(*this, ffFrame))
    {
        timerN_As->startTimer();
        internalStatus = AWAITING_FF_ACK;
        result = IN_PROGRESS;
        return result;
    }

    returnError(N_ERROR);
}

N_Result N_USData_Request_Runner::run_step_SF(const CANFrame* receivedFrame)
{
    if (receivedFrame != nullptr)
    {
        returnError(N_ERROR);
    }

    timerN_As->startTimer();

    CANFrame sfFrame = NewCANFrameDoCANCpp();
    sfFrame.identifier = nAi;

    sfFrame.data[0] = messageLength; // N_PCI_SF (0b0000xxxx) | messageLength (0bxxxxllll)
    memcpy(&sfFrame.data[1], messageData, messageLength); // Payload data

    sfFrame.data_length_code = messageLength + 1; // 1 byte for N_PCI_SF

    if (CanMessageACKQueue->writeFrame(*this, sfFrame))
    {
        internalStatus = AWAITING_SF_ACK;
        result = IN_PROGRESS;
        return result;
    }
    result = N_ERROR;
    return result;
}

N_Result N_USData_Request_Runner::run_step_FC(const CANFrame* receivedFrame, const bool firstFC)
{
    FlowStatus fs;
    uint8_t bs;
    STmin stM;
    if (parseFCFrame(receivedFrame, fs, bs, stM) != N_OK)
    {
        return result; // All the other error parameters are already set.
    }

    switch (fs)
    {
        case CONTINUE_TO_SEND:
        {
            blockSize = bs;
            cfSentInThisBlock = 0;
            stMin = stM;

            timerN_Bs->stopTimer();
            timerN_Cs->startTimer();

            result = IN_PROGRESS;
            internalStatus = SEND_CF;
            return result;
        }
        case WAIT:
        {
            // Restart N_Bs timer
            timerN_Bs->startTimer();
            internalStatus = AWAITING_FC;
            result = IN_PROGRESS;
            return result;
        }
        case OVERFLOW:
            if (firstFC)
            {
                returnError(N_BUFFER_OVFLW);
            }
            [[fallthrough]];
        default:
            returnError(N_INVALID_FS);
    }
}

bool N_USData_Request_Runner::awaitingMessage() const { return internalStatus == AWAITING_FC || internalStatus == AWAITING_FirstFC; }

uint32_t N_USData_Request_Runner::getNextTimeoutTime() const
{
    uint32_t timeoutAs = timerN_As->getStartTimeStamp() + N_As_TIMEOUT_MS;
    uint32_t timeoutBs = timerN_Bs->getStartTimeStamp() + N_Bs_TIMEOUT_MS;
    uint32_t timeoutCs = timerN_Cs->getStartTimeStamp() + getStMinInMs(stMin);
    uint32_t minTimeoutAsBs = MIN(timeoutAs, timeoutBs);
    return MIN(minTimeoutAsBs, timeoutCs);
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

    return nextRunTime;
}

void N_USData_Request_Runner::messageACKReceivedCallback(CANInterface::ACKResult success)
{
    if (!mutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        result = N_ERROR;
        internalStatus = ERROR;
        return;
    }

    switch (internalStatus)
    {
        case AWAITING_SF_ACK:
        {
            if (success == CANInterface::ACK_SUCCESS)
            {
                timerN_As->stopTimer();
                internalStatus = MESSAGE_SENT;
            }
            else
            {
                result = N_ERROR;
                internalStatus = ERROR;
            }
            break;
        }
        case AWAITING_FF_ACK:
        {
            if (success == CANInterface::ACK_SUCCESS)
            {
                internalStatus = AWAITING_FirstFC;
                timerN_As->stopTimer();
                timerN_Bs->startTimer();
            }
            else
            {
                result = N_ERROR;
                internalStatus = ERROR;
            }
            break;
        }
        case AWAITING_CF_ACK:
        {
            if (success == CANInterface::ACK_SUCCESS)
            {
                timerN_As->stopTimer();

                if (messageOffset == messageLength)
                {
                    timerN_As->stopTimer();
                    internalStatus = MESSAGE_SENT;
                }
                else if (cfSentInThisBlock == blockSize)
                {
                    internalStatus = AWAITING_FC;
                    timerN_Bs->startTimer();
                }
                else
                {
                    timerN_Cs->startTimer();
                    internalStatus = SEND_CF;
                }
            }
            else
            {
                result = N_ERROR;
                internalStatus = ERROR;
            }
            break;
        }
        default:
            assert(false && "Invalid internal status");
    }

    mutex->signal();
}

N_Result N_USData_Request_Runner::parseFCFrame(const CANFrame* receivedFrame, FlowStatus& fs, uint8_t& blcksize, STmin& stM)
{
    if (receivedFrame == nullptr)
    {
        returnError(N_ERROR);
    }

    timerN_Bs->stopTimer();
    timerN_Cs->startTimer();

    if (receivedFrame->identifier.N_TAtype != N_TATYPE_5_CAN_CLASSIC_29bit_Physical)
    {
        returnError(N_ERROR);
    }

    if (receivedFrame->data_length_code != FC_MESSAGE_LENGTH)
    {
        returnError(N_ERROR);
    }

    if ((receivedFrame->data[0] >> 4 & 0b00001111) != FC_CODE)
    {
        returnError(N_ERROR);
    }

    fs = static_cast<FlowStatus>(receivedFrame->data[0] & 0b00001111);
    if (fs >= INVALID_FS)
    {
        returnError(N_ERROR);
    }

    blcksize = receivedFrame->data[1];

    if (receivedFrame->data[2] <= 0x7F)
    {
        stM.unit = ms;
        stM.value = receivedFrame->data[2];
    }
    else if (receivedFrame->data[2] >= 0xF1 && receivedFrame->data[2] <= 0xF9)
    {
        stM.unit = usX100;
        stM.value = receivedFrame->data[2] & 0x0F;
    }
    else // Reserved values -> max stMin value
    {
        stM.unit = ms;
        stM.value = 127;
    }

    return N_OK;
}

N_AI N_USData_Request_Runner::getN_AI() const { return nAi; }

uint8_t* N_USData_Request_Runner::getMessageData() const { return messageData; }

uint32_t N_USData_Request_Runner::getMessageLength() const { return messageLength; }

N_Result N_USData_Request_Runner::getResult() const { return result; }

Mtype N_USData_Request_Runner::getMtype() const { return mType; }

N_USData_Request_Runner::RunnerType N_USData_Request_Runner::getRunnerType() const { return this->runnerType; }

const char* N_USData_Request_Runner::getTAG() const { return this->tag; }
