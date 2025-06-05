#include "LocalCANNetwork.h"
#include "OSInterface.h"

constexpr uint32_t maxSyncTimeMS = 100;

LocalCANNetworkCANInterface::LocalCANNetworkCANInterface(LocalCANNetwork* network, uint32_t nodeID, const char* tag)
{
    this->tag     = tag;
    this->network = network;
    this->nodeID  = nodeID;
}

uint32_t LocalCANNetworkCANInterface::frameAvailable()
{
    return network->frameAvailable(nodeID);
}

bool LocalCANNetworkCANInterface::readFrame(CANFrame* frame)
{
    // OSInterfaceLogDebug(tag, "Reading frame");
    return network->readFrame(nodeID, frame);
}

bool LocalCANNetworkCANInterface::writeFrame(CANFrame* frame)
{
    // OSInterfaceLogDebug(tag, "Writing frame with N_AI=%s: ", nAiToString(frame->identifier));
    return network->writeFrame(nodeID, frame);
}

bool LocalCANNetworkCANInterface::active()
{
    return network->active();
}

CANInterface::ACKResult LocalCANNetworkCANInterface::getWriteFrameACK()
{
    // OSInterfaceLogVerbose(tag, "Getting write frame ACK for node ID %u", nodeID);
    return network->getWriteFrameACK(nodeID);
}

uint32_t LocalCANNetworkCANInterface::getNodeID() const
{
    return nodeID;
}

LocalCANNetwork::LocalCANNetwork()
{
    this->accessMutex = LinuxOSInterface().osCreateMutex();
}

LocalCANNetwork::~LocalCANNetwork()
{
    delete accessMutex;
}

LocalCANNetworkCANInterface* LocalCANNetwork::newCANInterfaceConnection(const char* tag)
{
    if (accessMutex->wait(maxSyncTimeMS))
    {
        network.emplace_back();
        lastACKList.push_back(CANInterface::ACK_NONE);
        accessMutex->signal();
        return new LocalCANNetworkCANInterface(this, nextNodeID++, tag);
    }
    return nullptr;
}

bool LocalCANNetwork::writeFrame(const uint32_t emitterID, CANFrame* frame)
{
    if (active() && accessMutex->wait(maxSyncTimeMS))
    {
        if (checkNodeID(emitterID))
        {
            for (size_t i = 0; i < network.size(); i++)
            {
                auto* frames = &network[i];
                if (frames != &network[emitterID])
                {
                    frames->push_back(*frame);
                }
                else
                {
                    lastACKList[i] = CANInterface::ACK_SUCCESS; // Simulate successful write
                }
            }
            accessMutex->signal();
            return true;
        }
        accessMutex->signal();
    }
    return false;
}

bool LocalCANNetwork::peekFrame(uint32_t nodeID, CANFrame* frame) const
{
    if (active() && accessMutex->wait(maxSyncTimeMS))
    {
        if (checkNodeID(nodeID) && !network[nodeID].empty())
        {
            *frame = network[nodeID].front();
            accessMutex->signal();
            return true;
        }
        accessMutex->signal();
    }
    return false;
}

bool LocalCANNetwork::readFrame(uint32_t nodeID, CANFrame* frame)
{
    if (peekFrame(nodeID, frame) && accessMutex->wait(maxSyncTimeMS))
    {
        network[nodeID].pop_front();
        accessMutex->signal();
        return true;
    }
    return false;
}

uint32_t LocalCANNetwork::frameAvailable(uint32_t nodeID) const
{
    uint32_t res = 0;
    if (active() && accessMutex->wait(maxSyncTimeMS))
    {
        if (checkNodeID(nodeID))
        {
            res = network[nodeID].size();
        }
        accessMutex->signal();
    }
    return res;
}

bool LocalCANNetwork::active() const
{
    bool res = false;
    if (accessMutex->wait(maxSyncTimeMS))
    {
        res = allowActiveFlag && network.size() > 1;
        accessMutex->signal();
    }
    return res;
}

CANInterface::ACKResult LocalCANNetwork::getWriteFrameACK(uint32_t nodeID)
{
    CANInterface::ACKResult res = CANInterface::ACK_NONE;
    if (accessMutex->wait(maxSyncTimeMS))
    {
        res                 = lastACKList[nodeID];
        lastACKList[nodeID] = CANInterface::ACK_NONE;
        accessMutex->signal();
        // OSInterfaceLogVerbose("LocalCANNetwork", "Last ACK for node %u: %s", nodeID,
        // CANInterface::ackResultToString(res));
    }
    return res;
}

bool LocalCANNetwork::checkNodeID(uint32_t nodeID) const
{
    return nodeID < network.size();
}

void LocalCANNetwork::overrideActive(bool forceDisable)
{
    allowActiveFlag = !forceDisable;
}
