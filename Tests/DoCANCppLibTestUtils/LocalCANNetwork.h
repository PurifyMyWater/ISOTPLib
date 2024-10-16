#ifndef DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H
#define DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H

#include "CANShim.h"
#include <vector>
#include <list>

/**
 * @brief A local CAN network that can be used to test CANShim implementations
 * The ID of each node is the index of the node in the network vector
 */
class LocalCANNetwork
{
public:
    bool writeFrame(uint32_t emmiterID, CANFrame* frame);
    bool readFrame(uint32_t receiverID, CANFrame* frame);
    bool peekFrame(uint32_t receiverID, CANFrame* frame);
    uint32_t frameAvailable(uint32_t receiverID);
    bool active();

    CANShim* newCANShimConnection();

private:
    bool checkNodeID(uint32_t nodeID);
    std::vector<std::list<CANFrame>> network;
    uint32_t nextNodeID = 0;
};

/**
 * @brief A CANShim implementation that uses a LocalCANNetwork to simulate a CAN bus
 */
class LocalCANNetworkCANShim : public CANShim
{
public:
    uint32_t frameAvailable() override;
    bool readFrame(CANFrame* frame) override;
    bool writeFrame(CANFrame* frame) override;
    bool active() override;

    LocalCANNetworkCANShim(LocalCANNetwork* network, uint32_t nodeID);
private:
    LocalCANNetwork* network;
    uint32_t nodeID;
};

#endif //DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H
