#include "cassert"

#include "DoCANCpp.h"

#include <ranges>

#include "N_USData_Indication_Runner.h"
#include "N_USData_Request_Runner.h"

const char* DoCANCpp::TAG = "DoCANCpp";

DoCANCpp::DoCANCpp(const typeof(N_AI::N_SA) nSA, const uint32_t totalAvailableMemoryForRunners, const N_USData_confirm_cb_t N_USData_confirm_cb, const N_USData_indication_cb_t N_USData_indication_cb,
                   const N_USData_FF_indication_cb_t N_USData_FF_indication_cb, OSInterface& osInterface, CANInterface& canInterface, const uint8_t blockSize, const STmin stMin) :
    osInterface(osInterface), canInterface(canInterface), availableMemoryForRunners(totalAvailableMemoryForRunners, osInterface)
{
    this->CanMessageACKQueue = new CANMessageACKQueue(canInterface, osInterface);
    this->nSA = nSA;
    this->availableMemoryForRunners.set(totalAvailableMemoryForRunners);
    this->N_USData_confirm_cb = N_USData_confirm_cb;
    this->N_USData_indication_cb = N_USData_indication_cb;
    this->N_USData_FF_indication_cb = N_USData_FF_indication_cb;
    this->blockSize = blockSize;
    this->lastRunTime = 0;

    this->configMutex = this->osInterface.osCreateMutex();
    this->notStartedRunnersMutex = this->osInterface.osCreateMutex();
    this->runnersMutex = this->osInterface.osCreateMutex();

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

void DoCANCpp::setN_SA(const typeof(N_AI::N_SA) nSA)
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    this->nSA = nSA;
    configMutex->signal();
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
    STmin stM = this->stMin;
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

bool DoCANCpp::N_USData_request(const typeof(N_AI::N_TA) nTa, const N_TAtype_t nTaType, const uint8_t* messageData, const uint32_t length, const Mtype mType)
{
    bool result;
    N_AI nAI = DoCANCpp_N_AI_CONFIG(nTaType, nTa, getN_SA());
    N_USData_Runner* runner = new N_USData_Request_Runner(&result, nAI, availableMemoryForRunners, mType, messageData, length, osInterface, *CanMessageACKQueue);
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

void DoCANCpp::run_step(DoCANCpp* self)
{
    // The first part of the run_step is to check if the CAN is active, and more than DoCANCpp_RunPeriod_MS has passed since the last run.
    if (self->osInterface.osMillis() - self->lastRunTime > DoCANCpp_RunPeriod_MS)
    {
        self->lastRunTime = self->osInterface.osMillis();

        if (self->canInterface.active()) // TODO what happens if the CAN is not active and we have messages mid-transmission (in/out)?
        {
            // Get the configuration used in this run_step.
            self->configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
            typeof(N_AI::N_SA) nSA = self->nSA;
            std::unordered_set<typeof(N_AI::N_TA)> acceptedFunctionalN_TAs = self->acceptedFunctionalN_TAs;
            STmin stMin = self->stMin;
            uint8_t blockSize = self->blockSize;
            self->configMutex->signal();

            self->runnersMutex->wait(DoCANCpp_MaxTimeToWaitForRunnersSync_MS);

            // The second part of the run_step is to check if there are any runners in notStartedRunners, and move them to activeRunners.
            // ISO 15765-2 specifies that there should not be more than one message with the same N_AI being transmitted or received at the same time.
            // If that happens, leave the message in the notStartedRunners queue until the current message with this N_AI is processed.
            self->notStartedRunnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);

            auto it = self->notStartedRunners.begin();
            while (it != self->notStartedRunners.end())
            {
                if (!self->activeRunners.contains((*it)->getN_AI().N_AI))
                {
                    self->activeRunners.insert(std::make_pair((*it)->getN_AI().N_AI, *it));
                    it = self->notStartedRunners.erase(it); // Returns the next iterator if the current one is erased.
                }
                else
                {
                    ++it; // Move to the next iterator.
                }
            }

            self->notStartedRunnersMutex->signal();

            // The third part of the run_step is to check if a message is available, read it and check if this DoCANCpp object is interested in it.
            enum FrameStatus
            {
                frameNotAvailable = 0,
                frameAvailable,
                frameProcessed
            };
            FrameStatus frameStatus = frameNotAvailable;
            CANFrame frame;
            if (self->canInterface.frameAvailable())
            {
                self->canInterface.readFrame(&frame); // TODO que pasa si cambias N_SA y tienes mensajes pendientes
                if (frame.extd == 1 && frame.data_length_code > 0 && frame.data_length_code <= CAN_FRAME_MAX_DLC)
                {
                    if ((frame.identifier.N_TAtype == N_TATYPE_5_CAN_CLASSIC_29bit_Physical && frame.identifier.N_TA == nSA) ||
                        (frame.identifier.N_TAtype == N_TATYPE_6_CAN_CLASSIC_29bit_Functional && self->acceptedFunctionalN_TAs.contains(frame.identifier.N_TA)))
                    {
                        frameStatus = frameAvailable;
                    }
                }
            }

            // The fourth part of the run_step is to walk through all activeRunners checking if they need to run. If
            // they do, run them passing them the frame if it applies.
            for (auto runnerPair: self->activeRunners)
            {
                auto runner = runnerPair.second;
                assert(runner != nullptr);

                N_Result result = IN_PROGRESS; // If the runner does not run, do nothing in the switch below.

                if (frameStatus == frameAvailable && runner->awaitingMessage() && runner->getN_AI().N_AI == frame.identifier.N_AI) // If the runner has a message to process, do it immediately.
                {
                    // Run the runner with the frame.
                    result = runner->run_step(&frame);
                    frameStatus = frameProcessed;
                }
                else if (self->lastRunTime - runner->getNextRunTime() > 0) // If the runner is ready to run, do it.
                {
                    // Run the runner without the frame.
                    result = runner->run_step(nullptr);
                }

                // Check if the runner has finished
                switch (result)
                {
                    case IN_PROGRESS_FF:
                        assert(false && "N_Result::IN_PROGRESS_FF should not happen, as the runner has already received at least one frame (if it is an indication runner)");
                    case IN_PROGRESS:
                        break;
                    default:
                        self->finishedRunners.push_front(runner);
                        break;
                }
            }

            // The fifth part of the run_step is to check if a runner processed a message, and if no one did, start a new runner to handle it.
            if (frameStatus == frameAvailable)
            {
                N_USData_Runner* runner = new N_USData_Indication_Runner(frame.identifier, self->availableMemoryForRunners, blockSize, stMin, self->osInterface, *self->CanMessageACKQueue);
                switch (runner->run_step(&frame))
                {
                    case IN_PROGRESS:
                        assert(false && "N_Result::IN_PROGRESS should not happen, as the runner was just created");
                    case IN_PROGRESS_FF:
                        if (self->N_USData_FF_indication_cb != nullptr)
                        {
                            self->N_USData_FF_indication_cb(runner->getN_AI(), runner->getMessageLength(), runner->getMtype());
                        }
                        self->activeRunners.insert(std::make_pair(runner->getN_AI().N_AI, runner));
                        break;
                    default: // Single frame or error
                        self->finishedRunners.push_front(runner);
                        break;
                }
            }

            // The sixth part of the run_step is to run the callbacks for the finished runners and remove them from activeRunners and finishedRunners.
            for (const auto runner: self->finishedRunners)
            {
                assert(runner->getRunnerType() != N_USData_Runner::RunnerUnknownType);
                // Call the callbacks.
                if (runner->getRunnerType() == N_USData_Runner::RunnerRequestType)
                {
                    if (self->N_USData_confirm_cb != nullptr)
                    {
                        self->N_USData_confirm_cb(runner->getN_AI(), runner->getResult(), runner->getMtype());
                    }
                }
                else if (runner->getRunnerType() == N_USData_Runner::RunnerIndicationType)
                {
                    if (self->N_USData_indication_cb != nullptr)
                    {
                        const uint8_t* messageData = runner->getMessageData();
                        self->N_USData_indication_cb(runner->getN_AI(), messageData, runner->getMessageLength(), runner->getResult(), runner->getMtype());
                    }
                }

                // Remove the runner from activeRunners.
                self->activeRunners.erase(runner->getN_AI().N_AI);
                delete runner;
            }
            self->finishedRunners.clear();

            self->runnersMutex->signal();
        }
    }
}

bool DoCANCpp::updateRunners()
{
    runnersMutex->wait(DoCANCpp_MaxTimeToWaitForRunnersSync_MS);

    for (const auto runner: notStartedRunners)
    {
        if (!updateRunner(runner))
        {
            return false;
        }
    }

    for (const auto runner: activeRunners | std::views::values)
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
