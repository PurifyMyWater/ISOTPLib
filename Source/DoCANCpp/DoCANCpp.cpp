#include "cassert"

#include "DoCANCpp.h"

#include <ranges>

#include "N_USData_Indication_Runner.h"
#include "N_USData_Request_Runner.h"

DoCANCpp::DoCANCpp(const typeof(N_AI::N_SA) nSA, const uint32_t totalAvailableMemoryForRunners,
                   const N_USData_confirm_cb_t       N_USData_confirm_cb,
                   const N_USData_indication_cb_t    N_USData_indication_cb,
                   const N_USData_FF_indication_cb_t N_USData_FF_indication_cb, OSInterface& osInterface,
                   CANInterface& canInterface, const uint8_t blockSize, const STmin stMin) :
    osInterface(osInterface), canInterface(canInterface),
    availableMemoryForRunners(totalAvailableMemoryForRunners, osInterface)
{
    this->CanMessageACKQueue = new CANMessageACKQueue(canInterface, osInterface);
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
        OSInterfaceLogWarning(DoCANCpp::TAG, "N_USData_confirm_cb is nullptr");
    }
    if (this->N_USData_indication_cb == nullptr)
    {
        OSInterfaceLogWarning(DoCANCpp::TAG, "N_USData_indication_cb is nullptr");
    }
    if (this->N_USData_FF_indication_cb == nullptr)
    {
        OSInterfaceLogWarning(DoCANCpp::TAG, "N_USData_FF_indication_cb is nullptr");
    }
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
                                                          length, osInterface, *CanMessageACKQueue);
    if (!result)
    {
        delete runner;
        return false;
    }
    result = notStartedRunnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    notStartedRunners.push_front(runner);
    notStartedRunnersMutex->signal();
    return result;
}

void DoCANCpp::runFinishedRunnerCallbacks()
{
    // The sixth part of the runStep is to run the callbacks for the finished runners and remove them from
    // activeRunners and finishedRunners.
    for (const auto runner : this->finishedRunners)
    {
        // Call the callbacks.
        if (runner->getRunnerType() == N_USData_Runner::RunnerRequestType)
        {
            if (this->N_USData_confirm_cb != nullptr)
            {
                this->N_USData_confirm_cb(runner->getN_AI(), runner->getResult(), runner->getMtype());
            }
        }
        else if (runner->getRunnerType() == N_USData_Runner::RunnerIndicationType)
        {
            if (this->N_USData_indication_cb != nullptr)
            {
                const uint8_t* messageData = runner->getMessageData();
                this->N_USData_indication_cb(runner->getN_AI(), messageData, runner->getMessageLength(),
                                             runner->getResult(), runner->getMtype());
            }
        }
        else
        {
            OSInterfaceLogError(DoCANCpp::TAG, "Runner type is unknown");
        }

        // Remove the runner from activeRunners.
        this->activeRunners.erase(runner->getN_AI().N_AI);
        delete runner;
    }
    this->finishedRunners.clear();
}

template <std::ranges::input_range R> void DoCANCpp::runErrorCallbacks(R&& runners) // TODO test this function
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
            OSInterfaceLogError(DoCANCpp::TAG, "Runner type is unknown");
        }

        delete runner;
    }
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

    // The third part of the runStep is to check if a message is available, read it and check if this DoCANCpp
    // object is interested in it.
    enum FrameStatus
    {
        frameNotAvailable = 0,
        frameAvailable,
        frameProcessed
    };
    FrameStatus frameStatus = frameNotAvailable;
    CANFrame    frame;
    if (this->canInterface.frameAvailable()) // FIXME Time of check vs time of use problem
    {
        this->canInterface.readFrame(&frame);
        if (frame.extd == 1 && frame.data_length_code > 0 && frame.data_length_code <= CAN_FRAME_MAX_DLC)
        {
            if ((frame.identifier.N_TAtype == N_TATYPE_5_CAN_CLASSIC_29bit_Physical &&
                 frame.identifier.N_TA == this->nSA) ||
                (frame.identifier.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional &&
                 this->acceptedFunctionalN_TAs.contains(frame.identifier.N_TA)))
            {
                frameStatus = frameAvailable;
            }
        }
    }

    this->runnersMutex->wait(DoCANCpp_MaxTimeToWaitForRunnersSync_MS);

    // The fourth part of the runStep is to walk through all activeRunners checking if they need to run. If
    // they do, run them passing them the frame if it applies.
    for (auto runnerPair : this->activeRunners)
    {
        auto runner = runnerPair.second;
        assert(runner != nullptr);

        N_Result result = IN_PROGRESS; // If the runner does not run, do nothing in the switch below.

        if (frameStatus == frameAvailable && runner->awaitingMessage() &&
            runner->getN_AI().N_AI ==
                frame.identifier.N_AI) // If the runner has a message to process, do it immediately.
        {
            // Run the runner with the frame.
            result      = runner->run_step(&frame);
            frameStatus = frameProcessed;
        }
        else if (this->lastRunTime - runner->getNextRunTime() > 0) // If the runner is ready to run, do it.
        {
            // Run the runner without the frame.
            result = runner->run_step(nullptr);
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
                this->finishedRunners.push_front(runner);
                break;
        }
    }

    // The fifth part of the runStep is to check if a runner processed a message, and if no one did, start a
    // new runner to handle it.
    if (frameStatus == frameAvailable)
    {
        bool result;

        N_USData_Runner* runner =
            new N_USData_Indication_Runner(result, frame.identifier, this->availableMemoryForRunners, blockSize, stMin,
                                           this->osInterface, *this->CanMessageACKQueue);
        if (!result)
        {
            // TODO: handle this error (Out of memory)
        }

        switch (runner->run_step(&frame))
        {
            case IN_PROGRESS:
                assert(false && "N_Result::IN_PROGRESS should not happen, as the runner was just created");
            case IN_PROGRESS_FF:
                if (this->N_USData_FF_indication_cb != nullptr)
                {
                    this->N_USData_FF_indication_cb(runner->getN_AI(), runner->getMessageLength(), runner->getMtype());
                }
                this->activeRunners.insert(std::make_pair(runner->getN_AI().N_AI, runner));
                break;
            default: // Single frame or error
                this->finishedRunners.push_front(runner);
                break;
        }
    }

    runFinishedRunnerCallbacks();

    this->runnersMutex->signal();
}

void DoCANCpp::runStep(DoCANCpp* self)
{
    if (self == nullptr)
    {
        OSInterfaceLogError(DoCANCpp::TAG, "DoCANCpp is null");
        return;
    }

    // The first part of the runStep is to check if the CAN is active, and more than DoCANCpp_RunPeriod_MS has passed
    // since the last run.
    if (self->osInterface.osMillis() - self->lastRunTime > DoCANCpp_RunPeriod_MS)
    {
        self->lastRunTime = self->osInterface.osMillis();

        if (self->canInterface
                .active()) // TODO what happens if the CAN is not active and we have messages mid-transmission (in/out)?
        {
            self->runStepCanActive();
        }
        else
        {
            self->runStepCanInactive(); // TODO: avoid calling this function always, do it only once until can is active
                                        // again.
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
