#include "LocalCANNetwork.h"

constexpr uint32_t maxSyncTimeMS = 100;

LocalCANNetworkCANShim::LocalCANNetworkCANShim(LocalCANNetwork* network, uint32_t nodeID)
{
    this->network = network;
    this->nodeID = nodeID;
}

uint32_t LocalCANNetworkCANShim::frameAvailable() { return network->frameAvailable(nodeID); }

bool LocalCANNetworkCANShim::readFrame(CANFrame* frame) { return network->readFrame(nodeID, frame); }

bool LocalCANNetworkCANShim::writeFrame(CANFrame* frame) { return network->writeFrame(nodeID, frame); }

bool LocalCANNetworkCANShim::active() { return network->active(); }

CANShim::ACKResult LocalCANNetworkCANShim::getWriteFrameACK() { return network->getWriteFrameACK(); }

uint32_t LocalCANNetworkCANShim::getNodeID() const { return nodeID; }


LocalCANNetworkCANShim* LocalCANNetwork::newCANShimConnection()
{
    if (accessMutex->wait(maxSyncTimeMS))
    {
        network.emplace_back();
        accessMutex->signal();
        return new LocalCANNetworkCANShim(this, nextNodeID++);
    }
    return nullptr;
}

bool LocalCANNetwork::writeFrame(uint32_t emitterID, CANFrame* frame)
{
    if (active() && accessMutex->wait(maxSyncTimeMS))
    {
        if (checkNodeID(emitterID))
        {
            for (auto& frames: network)
            {
                if (&frames != &network[emitterID])
                {
                    frames.push_back(*frame);
                }
            }
            accessMutex->signal();
            return true;
        }
        accessMutex->signal();
    }
    return false;
}

bool LocalCANNetwork::peekFrame(uint32_t receiverID, CANFrame* frame) const
{
    if (active() && accessMutex->wait(maxSyncTimeMS))
    {
        if (checkNodeID(receiverID) && !network[receiverID].empty())
        {
            *frame = network[receiverID].front();
            accessMutex->signal();
            return true;
        }
        accessMutex->signal();
    }
    return false;
}

bool LocalCANNetwork::readFrame(uint32_t receiverID, CANFrame* frame)
{
    if (peekFrame(receiverID, frame) && accessMutex->wait(maxSyncTimeMS))
    {
        lastACK = CANShim::ACK_SUCCESS;
        network[receiverID].pop_front();
        accessMutex->signal();
        return true;
    }
    return false;
}

uint32_t LocalCANNetwork::frameAvailable(uint32_t receiverID) const
{
    uint32_t res = 0;
    if (active() && accessMutex->wait(maxSyncTimeMS))
    {
        if (checkNodeID(receiverID))
        {
            res = network[receiverID].size();
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

CANShim::ACKResult LocalCANNetwork::getWriteFrameACK()
{
    CANShim::ACKResult res = CANShim::ACK_NONE;
    if (accessMutex->wait(maxSyncTimeMS))
    {
        res = lastACK;
        lastACK = CANShim::ACK_NONE;
        accessMutex->signal();
    }
    return res;
}

bool LocalCANNetwork::checkNodeID(uint32_t nodeID) const { return nodeID < network.size(); }

void LocalCANNetwork::overrideActive(bool forceDisable) { allowActiveFlag = !forceDisable; }
