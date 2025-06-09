#include "cassert"

#include "DoCANCpp.h"

#include <ranges>

#include "N_USData_Indication_Runner.h"
#include "N_USData_Request_Runner.h"

DoCANCpp::DoCANCpp(const typeof(N_AI::N_SA) nSA, const uint32_t totalAvailableMemoryForRunners,
                   const N_USData_confirm_cb_t       N_USData_confirm_cb,
                   const N_USData_indication_cb_t    N_USData_indication_cb,
                   const N_USData_FF_indication_cb_t N_USData_FF_indication_cb, OSInterface& osInterface,
                   CANInterface& canInterface, const uint8_t blockSize, const STmin stMin, const char* tag) :
    osInterface(osInterface), canInterface(canInterface),
    availableMemoryForRunners(totalAvailableMemoryForRunners, osInterface)
{
    this->tag = tag;

    this->queueTag = nullptr;
    ASSERT_SAFE(populateQueueTag(), == true);

    this->canMessageAckQueue = new CANMessageACKQueue(canInterface, osInterface, this->queueTag);
    this->nSA                = nSA;
    this->availableMemoryForRunners.set(totalAvailableMemoryForRunners);
    this->N_USData_confirm_cb       = N_USData_confirm_cb;
    this->N_USData_indication_cb    = N_USData_indication_cb;
    this->N_USData_FF_indication_cb = N_USData_FF_indication_cb;
    this->blockSize                 = blockSize;
    this->lastRunTime               = 0;

    this->configMutex            = this->osInterface.osCreateMutex();
    this->notStartedRunnersMutex = this->osInterface.osCreateMutex();
    this->runnersMutex           = this->osInterface.osCreateMutex();

    assert(this->configMutex != nullptr && this->notStartedRunnersMutex != nullptr && "Mutex creation failed");

    ASSERT_SAFE(setSTmin(stMin), == true);

    if (this->N_USData_confirm_cb == nullptr)
    {
        OSInterfaceLogWarning(this->tag, "N_USData_confirm_cb is nullptr");
    }
    if (this->N_USData_indication_cb == nullptr)
    {
        OSInterfaceLogWarning(this->tag, "N_USData_indication_cb is nullptr");
    }
    if (this->N_USData_FF_indication_cb == nullptr)
    {
        OSInterfaceLogWarning(this->tag, "N_USData_FF_indication_cb is nullptr");
    }
}

const char* DoCANCpp::getTag() const
{
    return this->tag;
}

DoCANCpp::~DoCANCpp()
{
    if (this->queueTag != nullptr)
    {
        this->osInterface.osFree(this->queueTag);
    }
    delete this->canMessageAckQueue;

    for (auto& runner : this->notStartedRunners)
    {
        delete runner;
    }
    for (auto& runner : this->activeRunners | std::views::values)
    {
        delete runner;
    }
    for (auto& runner : this->finishedRunners)
    {
        delete runner;
    }

    delete this->configMutex;
    delete this->notStartedRunnersMutex;
    delete this->runnersMutex;
}

bool DoCANCpp::populateQueueTag()
{
    int queueTagSize = snprintf(nullptr, 0, "%s-%s", tag, "ACKQueue");
    this->queueTag   = static_cast<char*>(osInterface.osMalloc(queueTagSize + 1));
    if (this->queueTag == nullptr)
    {
        OSInterfaceLogError(tag, "Failed to allocate memory for CANMessageACKQueue tag");
        return false;
    }
    snprintf(this->queueTag, queueTagSize + 1, "%s-%s", tag, "ACKQueue");
    return true;
}

typeof(N_AI::N_SA) DoCANCpp::getN_SA() const
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    typeof(N_AI::N_SA) NSA = this->nSA;
    configMutex->signal();
    return NSA;
}

void DoCANCpp::addAcceptedFunctionalN_TA(const typeof(N_AI::N_TA) nTA)
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    this->acceptedFunctionalN_TAs.insert(nTA);
    configMutex->signal();
}

bool DoCANCpp::removeAcceptedFunctionalN_TA(const typeof(N_AI::N_TA) nTA)
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    bool res = this->acceptedFunctionalN_TAs.erase(nTA) == 1;
    configMutex->signal();
    return res;
}

bool DoCANCpp::hasAcceptedFunctionalN_TA(const typeof(N_AI::N_TA) nTA)
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    bool res = this->acceptedFunctionalN_TAs.contains(nTA);
    configMutex->signal();
    return res;
}

uint8_t DoCANCpp::getBlockSize() const
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    uint8_t bs = this->blockSize;
    configMutex->signal();
    return bs;
}

bool DoCANCpp::setBlockSize(const uint8_t bs)
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    this->blockSize = bs;
    configMutex->signal();

    return updateRunners();
}

STmin DoCANCpp::getSTmin() const
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    const STmin stM = this->stMin;
    configMutex->signal();
    return stM;
}

bool DoCANCpp::setSTmin(const STmin stMin)
{
    if ((stMin.unit == ms && stMin.value > 127) || (stMin.unit == usX100 && (stMin.value < 1 || stMin.value > 9)))
    {
        return false;
    }

    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    this->stMin = stMin;
    configMutex->signal();

    return updateRunners();
}

bool DoCANCpp::N_USData_request(const typeof(N_AI::N_TA) nTa, const N_TAtype_t nTaType, const uint8_t* messageData,
                                const uint32_t length, const Mtype mType)
{
    bool             result;
    N_AI             nAI    = DoCANCpp_N_AI_CONFIG(nTaType, nTa, getN_SA());
    N_USData_Runner* runner = new N_USData_Request_Runner(result, nAI, availableMemoryForRunners, mType, messageData,
                                                          length, osInterface, *canMessageAckQueue);
    if (!result)
    {
        delete runner;
        return false;
    }
    if (notStartedRunnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS))
    {
        notStartedRunners.push_back(runner);
        notStartedRunnersMutex->signal();
        return true;
    }
    return false;
}

void DoCANCpp::runFinishedRunnerCallbacks()
{
    for (const auto runner : this->finishedRunners)
    {
        OSInterfaceLogInfo(this->tag, "Runner %s finished with result %s", runner->getTAG(),
                           N_ResultToString(runner->getResult()));
        // Call the callbacks.
        if (runner->getRunnerType() == N_USData_Runner::RunnerRequestType)
        {
            if (this->N_USData_confirm_cb != nullptr)
            {
                OSInterfaceLogInfo(this->tag, "Calling N_USData_confirm_cb of runner %s", runner->getTAG());
                this->N_USData_confirm_cb(runner->getN_AI(), runner->getResult(), runner->getMtype());
            }
        }
        else if (runner->getRunnerType() == N_USData_Runner::RunnerIndicationType)
        {
            if (this->N_USData_indication_cb != nullptr)
            {
                OSInterfaceLogInfo(this->tag, "Calling N_USData_indication_cb of runner %s", runner->getTAG());
                const uint8_t* messageData = runner->getMessageData();
                this->N_USData_indication_cb(runner->getN_AI(), messageData, runner->getMessageLength(),
                                             runner->getResult(), runner->getMtype());
            }
        }
        else
        {
            OSInterfaceLogError(this->tag, "Runner type is unknown");
        }

        // Remove the runner from activeRunners.
        this->activeRunners.erase(runner->getN_AI().N_AI);
        canMessageAckQueue->removeFromQueue(runner->getN_AI());
        delete runner;
    }
    this->finishedRunners.clear();
}

template <std::ranges::input_range R> void DoCANCpp::runErrorCallbacks(R&& runners)
{
    for (const auto runner : runners)
    {
        // Call the callbacks.
        if (runner->getRunnerType() == N_USData_Runner::RunnerRequestType)
        {
            if (this->N_USData_confirm_cb != nullptr)
            {
                this->N_USData_confirm_cb(runner->getN_AI(), N_ERROR, runner->getMtype());
            }
        }
        else if (runner->getRunnerType() == N_USData_Runner::RunnerIndicationType)
        {
            if (this->N_USData_indication_cb != nullptr)
            {
                this->N_USData_indication_cb(runner->getN_AI(), nullptr, 0, N_ERROR, Mtype_Unknown);
            }
        }
        else
        {
            OSInterfaceLogError(this->tag, "Runner type is unknown");
        }

        delete runner;
    }
}

void DoCANCpp::startRunners()
{
    // The second part of the runStep is to check if there are any runners in notStartedRunners, and move them
    // to activeRunners. ISO 15765-2 specifies that there should not be more than one message with the same N_AI
    // being transmitted or received at the same time. If that happens, leave the message in the
    // notStartedRunners queue until the current message with this N_AI is processed.
    this->runnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    this->notStartedRunnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);

    auto it = this->notStartedRunners.begin();
    while (it != this->notStartedRunners.end())
    {
        if (!this->activeRunners.contains((*it)->getN_AI().N_AI))
        {
            this->activeRunners.insert(std::make_pair((*it)->getN_AI().N_AI, *it));
            it = this->notStartedRunners.erase(it); // Returns the next iterator if the current one is erased.
        }
        else
        {
            ++it; // Move to the next iterator.
        }
    }

    this->notStartedRunnersMutex->signal();
    this->runnersMutex->signal();
}

void DoCANCpp::getFrameIfAvailable(FrameStatus& frameStatus, CANFrame& frame) const
{
    frameStatus = frameNotAvailable;
    if (this->canInterface.frameAvailable())
    {
        this->canInterface.readFrame(&frame);
        if (frame.extd == 1 && frame.data_length_code > 0 && frame.data_length_code <= CAN_FRAME_MAX_DLC)
        {
            OSInterfaceLogVerbose(this->tag, "Received frame: %s", frameToString(frame));
            if ((frame.identifier.N_TAtype == N_TATYPE_5_CAN_CLASSIC_29bit_Physical &&
                 frame.identifier.N_TA == this->nSA) ||
                (frame.identifier.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional &&
                 this->acceptedFunctionalN_TAs.contains(frame.identifier.N_TA)))
            {
                OSInterfaceLogDebug(this->tag, "Received frame for this DoCANCpp instance: %s", frameToString(frame));
                frameStatus = frameAvailable;
            }
        }
    }
}

void DoCANCpp::runRunners(FrameStatus& frameStatus, CANFrame frame)
{
    for (auto runner : this->activeRunners | std::views::values)
    {
        N_Result result = IN_PROGRESS; // If the runner does not run, do nothing in the switch below.

        if (frameStatus == frameAvailable &&
            runner->isThisFrameForMe(frame)) // If the runner has a message to process, do it immediately.
        {
            OSInterfaceLogDebug(this->tag, "Runner %s is processing frame: %s", runner->getTAG(), frameToString(frame));
            // Run the runner with the frame.
            result      = runner->runStep(&frame);
            frameStatus = frameProcessed;
        }
        else if (this->lastRunTime > runner->getNextRunTime()) // If the runner is ready to run, do it.
        {
            OSInterfaceLogDebug(this->tag, "Runner %s is running without frame", runner->getTAG());
            // Run the runner without the frame.
            result = runner->runStep(nullptr);
        }

        // Check if the runner has finished
        switch (result)
        {
            case IN_PROGRESS_FF:
                assert(false && "N_Result::IN_PROGRESS_FF should not happen, as the runner has already "
                                "received at least one frame (if it is an indication runner)");
            case IN_PROGRESS:
                break;
            default:
                this->finishedRunners.push_back(runner);
                break;
        }
    }
}

void DoCANCpp::createRunnerForMessage(STmin stMin, uint8_t blockSize, FrameStatus frameStatus, CANFrame frame)
{
    if (frameStatus == frameAvailable)
    {
        bool result;

        N_USData_Runner* runner =
            new N_USData_Indication_Runner(result, frame.identifier, this->availableMemoryForRunners, blockSize, stMin,
                                           this->osInterface, *this->canMessageAckQueue);
        if (runner == nullptr)
        {
            OSInterfaceLogError(this->tag, "Failed to create a new runner");
        }
        else if (!result)
        {
            OSInterfaceLogError(this->tag, "Failed to create a new runner");
            delete runner;
        }
        else
        {
            switch (runner->runStep(&frame))
            {
                case IN_PROGRESS:
                    assert(false && "N_Result::IN_PROGRESS should not happen, as the runner was just created");
                case IN_PROGRESS_FF:
                    if (this->N_USData_FF_indication_cb != nullptr)
                    {
                        OSInterfaceLogInfo(this->tag, "Calling N_USData_FF_indication_cb of runner %s",
                                           runner->getTAG());
                        this->N_USData_FF_indication_cb(runner->getN_AI(), runner->getMessageLength(),
                                                        runner->getMtype());
                    }
                    this->activeRunners.insert(std::make_pair(runner->getN_AI().N_AI, runner));
                    break;
                default: // Single frame or error
                    this->finishedRunners.push_front(runner);
                    break;
            }
        }
    }
}
void DoCANCpp::runStepCanActive()
{
    // Get the configuration used in this runStep.
    this->configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    std::unordered_set<typeof(N_AI::N_TA)> acceptedFunctionalN_TAs = this->acceptedFunctionalN_TAs;
    STmin                                  stMin                   = this->stMin;
    uint8_t                                blockSize               = this->blockSize;
    this->configMutex->signal();

    // The second part of the runStep is to check if there are any runners in notStartedRunners, and move them
    // to activeRunners. ISO 15765-2 specifies that there should not be more than one message with the same N_AI
    // being transmitted or received at the same time. If that happens, leave the message in the
    // notStartedRunners queue until the current message with this N_AI is processed.
    startRunners();

    // The third part of the runStep is to check if a message is available, read it and check if this DoCANCpp
    // object is interested in it.
    FrameStatus frameStatus;
    CANFrame    frame;
    getFrameIfAvailable(frameStatus, frame);

    this->runnersMutex->wait(DoCANCpp_MaxTimeToWaitForRunnersSync_MS);

    // The fourth part of the runStep is to walk through all activeRunners checking if they need to run. If
    // they do, run them passing them the frame if it applies.
    runRunners(frameStatus, frame);

    // The fifth part of the runStep is to check if a runner processed a message, and if no one did, start a
    // new runner to handle it.
    createRunnerForMessage(stMin, blockSize, frameStatus, frame);

    // The sixth part of the runStep is to run the callbacks for the finished runners and remove them from
    // activeRunners and finishedRunners.
    runFinishedRunnerCallbacks();

    this->runnersMutex->signal();
}

void DoCANCpp::runStepCanInactive()
{
    this->notStartedRunnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);

    runErrorCallbacks(this->notStartedRunners);
    this->notStartedRunners.clear();

    this->notStartedRunnersMutex->signal();
    this->runnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);

    runErrorCallbacks(this->activeRunners | std::views::values);
    this->activeRunners.clear();

    runFinishedRunnerCallbacks();

    this->runnersMutex->signal();
}

void DoCANCpp::runStep()
{
    // The first part of the runStep is to check if the CAN is active, and more than DoCANCpp_RunPeriod_MS has passed
    // since the last run.
    if (const uint32_t millis = this->osInterface.osMillis(); millis - this->lastRunTime > DoCANCpp_RunPeriod_MS)
    {
        this->lastRunTime = millis;

        if (this->canInterface.active())
        {
            this->runStepCanActive();
        }
        else
        {
            this->runStepCanInactive(); // TODO: avoid calling this function always, do it only once until can is active
                                        // again.
        }
    }
}
void DoCANCpp::canMessageACKQueueRunStep() const
{
    static uint32_t ACKlastRunTime = 0;
    if (this->osInterface.osMillis() - ACKlastRunTime > DoCANCpp_RunPeriod_ACKQueue_MS)
    {
        ACKlastRunTime = this->osInterface.osMillis();
        if (canMessageAckQueue != nullptr)
        {
            canMessageAckQueue->runStep();
        }
    }
}

bool DoCANCpp::updateRunners()
{
    notStartedRunnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);

    for (const auto runner : notStartedRunners)
    {
        if (!updateRunner(runner))
        {
            return false;
        }
    }

    notStartedRunnersMutex->signal();

    runnersMutex->wait(DoCANCpp_MaxTimeToWaitForRunnersSync_MS);

    for (const auto runner : activeRunners | std::views::values)
    {
        if (!updateRunner(runner))
        {
            return false;
        }
    }
    runnersMutex->signal();
    return true;
}

bool DoCANCpp::updateRunner(N_USData_Runner* runner) const
{
    if (runner->getRunnerType() == N_USData_Runner::RunnerIndicationType)
    {
        const auto indicationRunner = dynamic_cast<N_USData_Indication_Runner*>(runner);

        if (!indicationRunner->setBlockSize(blockSize))
        {
            return false;
        }
        return indicationRunner->setSTmin(stMin);
    }
    return true;
}
