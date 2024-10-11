#include "cassert"

#include "DoCANCpp.h"
#include "N_USData_Request_Runner.h"
#include "N_USData_Indication_Runner.h"

const char* DoCANCpp::TAG = "DoCANCpp";

DoCANCpp::DoCANCpp(typeof(N_AI::N_SA) nSA, uint32_t totalAvailableMemoryForRunners, N_USData_confirm_cb_t N_USData_confirm_cb, N_USData_indication_cb_t N_USData_indication_cb, N_USData_FF_indication_cb_t N_USData_FF_indication_cb, OSShim& osShim, CANShim& canShim, uint8_t blockSize, STmin stMin)
{
    this->osShim = &osShim;
    this->canShim = &canShim;
    this->nSA = nSA;
    this->availableMemoryForRunners = totalAvailableMemoryForRunners;
    this->N_USData_confirm_cb = N_USData_confirm_cb;
    this->N_USData_indication_cb = N_USData_indication_cb;
    this->N_USData_FF_indication_cb = N_USData_FF_indication_cb;
    this->blockSize = blockSize;
    this->stMin.value = stMin.value;
    this->stMin.unit = stMin.unit;
    this->lastRunTime = 0;

    this->configMutex = this->osShim->createMutex();
    this->notStartedRunnersMutex = this->osShim->createMutex();
}

typeof(N_AI::N_SA) DoCANCpp::getN_SA() const
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    typeof(N_AI::N_SA) NSA = this->nSA;
    configMutex->signal();
    return NSA;
}

void DoCANCpp::setN_SA(typeof(N_AI::N_SA) NSA)
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    this->nSA = NSA;
    configMutex->signal();
}

void DoCANCpp::addAcceptedFunctionalN_TA(typeof(N_AI::N_TA) nTA)
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    this->acceptedFunctionalN_TAs.insert(nTA);
    configMutex->signal();
}

bool DoCANCpp::removeAcceptedFunctionalN_TA(typeof(N_AI::N_TA) nTA)
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    bool res = this->acceptedFunctionalN_TAs.erase(nTA) == 1;
    configMutex->signal();
    return res;
}

bool DoCANCpp::hasAcceptedFunctionalN_TA(typeof(N_AI::N_TA) nTA)
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

void DoCANCpp::setBlockSize(uint8_t bs)
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    this->blockSize = bs;
    configMutex->signal();
}

DoCANCpp::STmin DoCANCpp::getSTmin() const
{
    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    STmin stM = this->stMin;
    configMutex->signal();
    return stM;
}

bool DoCANCpp::setSTmin(DoCANCpp::STmin stM)
{
    if((stM.unit == ms && stM.value > 127) || (stM.unit == us && (stM.value < 100 || stM.value > 900)))
    {
        return false;
    }

    configMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    this->stMin = stM;
    configMutex->signal();
    return true;
}

void DoCANCpp::N_USData_request(typeof(N_AI::N_TA) nTa, N_TAtype_t nTaType, uint8_t* messageData, uint32_t length, Mtype mtype)
{
    N_AI nAI = DoCANCpp_N_AI_CONFIG(nTaType, nTa, getN_SA());
    N_USData_Runner* runner = new N_USData_Request_Runner(nAI, mtype, messageData, length, osShim, canShim);
    notStartedRunnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);
    notStartedRunners.push_front(runner);
    notStartedRunnersMutex->signal();
}

void DoCANCpp::run_step(DoCANCpp* self)
{
    // The first part of the run_step is to check if the CAN is active, and more than DoCANCpp_RunPeriod_MS has passed since the last run.
    if(self->osShim->millis() - self->lastRunTime > DoCANCpp_RunPeriod_MS)
    {
        self->lastRunTime = self->osShim->millis(); // TODO count from the start or the end of the function call?

        if(self->canShim->active())
        {
            // The second part of the run_step is to check if there are any runners in notStartedRunners, and move them to activeRunners.
            // ISO 15765-2 specifies that there should not be more than one message with the same N_AI being transmitted or received at the same time.
            // If that happens, leave the message in the notStartedRunners queue until the current message with this N_AI is processed.
            self->notStartedRunnersMutex->wait(DoCANCpp_MaxTimeToWaitForSync_MS);

            auto it = self->notStartedRunners.begin();
            while (it != self->notStartedRunners.end())
            {
                if(! self->activeRunners.contains((*it)->getN_AI().N_AI))
                {
                    self->activeRunners.insert(std::make_pair((*it)->getN_AI().N_AI, (*it)));
                    it = self->notStartedRunners.erase(it); // Returns the next iterator if the current one is erased.
                }
                else
                {
                    ++it; // Move to the next iterator.
                }
            }

            for(auto runner : self->notStartedRunners)
            {
                self->activeRunners.insert(std::make_pair(runner->getN_AI().N_AI, runner));
            }
            self->notStartedRunners.clear();
            self->notStartedRunnersMutex->signal();

            // The third part of the run_step is to check if a message is available, read it and check if this DoCANCpp object is interested in it.
            enum FrameStatus {frameNotAvailable = 0, frameAvailable, frameProcessed};
            FrameStatus frameStatus = frameNotAvailable;
            CANShim::CANFrame frame;
            if(self->canShim->frameAvailable())
            {
                self->canShim->readFrame(&frame);
                if((frame.identifier.N_TAtype == CAN_CLASSIC_29bit_Physical && frame.identifier.N_TA == self->nSA) || (frame.identifier.N_TAtype == CAN_CLASSIC_29bit_Functional && self->acceptedFunctionalN_TAs.contains(frame.identifier.N_TA)))
                {
                    frameStatus = frameAvailable;
                }
            }

            // The fourth part of the run_step is to walk through all activeRunners checking if they need to run. If they do, run them passing them the frame if it applies.
            for(auto runnerPair : self->activeRunners)
            {
                auto runner = runnerPair.second;
                // If the runner has pending actions to run
                if(self->lastRunTime - runner->getNextRunTime() > 0)
                {
                    N_Result result = N_Result::N_ERROR;
                    // If a message is available, and the runner is waiting for it.
                    if(frameStatus == frameAvailable && runner->awaitingMessage() && runner->getN_AI().N_AI == frame.identifier.N_AI)
                    {
                        // Run the runner with the frame.
                        result = runner->run_step(&frame);
                        frameStatus = frameProcessed;
                    }
                    else
                    {
                        // Run the runner without the frame.
                        result = runner->run_step(nullptr);
                    }

                    // Check if the runner has finished
                    switch (result)
                    {
                        case N_Result::IN_PROGRESS_FF:
                            self->N_USData_FF_indication_cb(runner->getN_AI(), runner->getMessageLength(), runner->getMtype());
                            [[fallthrough]]; // Indicate intentional fall-through
                        case N_Result::IN_PROGRESS:
                            break;
                        default:
                            self->finishedRunners.push_front(runner);
                            break;
                    }
                }
            }

            // The fifth part of the run_step is to check if a runner processed a message, and if no one did, start a new runner to handle it.
            if(frameStatus == frameAvailable)
            {
                N_USData_Runner* runner = new N_USData_Indication_Runner(frame.identifier, &self->availableMemoryForRunners, self->osShim, self->canShim);
                N_Result result = runner->run_step(&frame);
                switch (result)
                {
                    case N_Result::IN_PROGRESS_FF:
                        self->N_USData_FF_indication_cb(runner->getN_AI(), runner->getMessageLength(), runner->getMtype());
                        [[fallthrough]]; // Indicate intentional fall-through
                    case N_Result::IN_PROGRESS:
                        self->activeRunners.insert(std::make_pair(runner->getN_AI().N_AI, runner));
                        break;
                    default:
                        self->finishedRunners.push_front(runner);
                        break;
                }
            }

            // The sixth part of the run_step is to run the callbacks for the finished runners and remove them from activeRunners and finishedRunners.
            for(auto runner : self->finishedRunners)
            {
                assert(runner->getRunnerType() != N_USData_Runner::RunnerUnknownType);
                // Call the callbacks.
                if(runner->getRunnerType() == N_USData_Runner::RunnerRequestType)
                {
                    self->N_USData_confirm_cb(runner->getN_AI(), runner->getResult(), runner->getMtype());
                }
                else if(runner->getRunnerType() == N_USData_Runner::RunnerIndicationType)
                {
                    self->N_USData_indication_cb(runner->getN_AI(), runner->getMessageData(), runner->getMessageLength(), runner->getResult(), runner->getMtype());
                }

                // Remove the runner from activeRunners.
                self->activeRunners.erase(runner->getN_AI().N_AI);
                delete runner;
            }
            self->finishedRunners.clear();
        }
    }
}
