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

uint32_t LocalCANNetworkCANShim::getNodeID()
{
    return nodeID;
}

bool LocalCANNetwork::writeFrame(uint32_t emmiterID, CANFrame* frame)
{
    return false;
}

bool LocalCANNetwork::readFrame(uint32_t receiverID, CANFrame* frame)
{
    return false;
}

bool LocalCANNetwork::peekFrame(uint32_t receiverID, CANFrame* frame)
{
    return false;
}

uint32_t LocalCANNetwork::frameAvailable(uint32_t receiverID)
{
    return 0;
}

bool LocalCANNetwork::active()
{
    return false;
}

LocalCANNetworkCANShim* LocalCANNetwork::newCANShimConnection()
{
    return nullptr;
}

bool LocalCANNetwork::checkNodeID(uint32_t nodeID)
{
    return nodeID < network.size();
}
