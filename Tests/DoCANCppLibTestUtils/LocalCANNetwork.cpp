#include "LocalCANNetwork.h"

LocalCANNetworkCANShim::LocalCANNetworkCANShim(LocalCANNetwork* network, uint32_t nodeID)
{
    this->network = network;
    this->nodeID = nodeID;
}

uint32_t LocalCANNetworkCANShim::frameAvailable()
{
    return network->frameAvailable(nodeID);
}

bool LocalCANNetworkCANShim::readFrame(CANFrame* frame)
{
    return network->readFrame(nodeID, frame);
}

bool LocalCANNetworkCANShim::writeFrame(CANFrame* frame)
{
    return network->writeFrame(nodeID, frame);
}

bool LocalCANNetworkCANShim::active()
{
    return network->active();
}

uint32_t LocalCANNetworkCANShim::getNodeID() const
{
    return nodeID;
}



LocalCANNetworkCANShim* LocalCANNetwork::newCANShimConnection()
{
    network.emplace_back();
    return new LocalCANNetworkCANShim(this, nextNodeID++);
}

bool LocalCANNetwork::writeFrame(uint32_t emitterID, CANFrame* frame)
{
    if(active() && checkNodeID(emitterID))
    {
        for(auto& frames : network)
        {
            if(&frames != &network[emitterID])
            {
                frames.push_back(*frame);
            }
        }
        return true;
    }
    return false;
}

bool LocalCANNetwork::peekFrame(uint32_t receiverID, CANFrame* frame)
{
    if(active() && checkNodeID(receiverID) && !network[receiverID].empty())
    {
        *frame = network[receiverID].front();
        return true;
    }
    return false;
}

bool LocalCANNetwork::readFrame(uint32_t receiverID, CANFrame* frame)
{
    if(peekFrame(receiverID, frame))
    {
        network[receiverID].pop_front();
        return true;
    }
    return false;
}

uint32_t LocalCANNetwork::frameAvailable(uint32_t receiverID)
{
    return active() && checkNodeID(receiverID) ? network[receiverID].size() : 0;
}

bool LocalCANNetwork::active()
{
    return network.size() > 1;
}

bool LocalCANNetwork::checkNodeID(uint32_t nodeID)
{
    return nodeID < network.size();
}
