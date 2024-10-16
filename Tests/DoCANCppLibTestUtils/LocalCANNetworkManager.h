#ifndef DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H
#define DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H

#include "CANShim.h"
#include <unordered_map>
#include <list>

class LocalCANNetwork
{
public:
    void writeFrame(uint32_t emmiterID, CANShim::CANFrame* frame);
    bool readFrame(uint32_t receiverID, CANShim::CANFrame* frame);
    uint32_t frameAvailable(uint32_t receiverID);
    bool active();

    uint32_t newNode();

private:
    std::unordered_map<uint32_t, std::list<CANShim::CANFrame>> network;
};

class LocalCANNetworkCANShim : public CANShim
{
public:
    uint32_t frameAvailable();
    bool readFrame(CANFrame* frame);
    bool writeFrame(CANFrame* frame);
    bool active();

    LocalCANNetworkCANShim(LocalCANNetwork* network, uint32_t nodeID);
private:
    LocalCANNetwork* network;
    uint32_t nodeID;

};


class LocalCANNetworkManager
{
    CANShim* newCANShimConnection();
};


#endif //DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H
