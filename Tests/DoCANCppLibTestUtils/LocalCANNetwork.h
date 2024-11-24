#ifndef DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H
#define DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H

#include <list>
#include <vector>
#include "CANShim.h"
#include "LinuxOSShim.h"

class LocalCANNetworkCANShim;

/**
 * @brief A local CAN network that can be used to test CANShim implementations
 * To use it, call newCANShimConnection() to create a new CANShim connection to the network, and use the CANShim as you would use normally
 */
class LocalCANNetwork
{
public:
    /**
     * @brief Create a new CANShim instance connected to the network
     * @return A new CANShim instance connected to the network
     */
    LocalCANNetworkCANShim* newCANShimConnection();

    /**
     * @brief Write a frame to the network (Internal use only)
     * @param emitterID The ID of the node that is emitting the frame
     * @param frame The frame to write
     * @return True if the frame was written successfully, false otherwise
     */
    bool writeFrame(uint32_t emitterID, CANFrame* frame);

    /**
     * @brief Read a frame from the network (Internal use only)
     * @param receiverID The ID of the node that is receiving the frame
     * @param frame The frame to read
     * @return True if a frame was read successfully, false otherwise
     */
    bool readFrame(uint32_t receiverID, CANFrame* frame);

    /**
     * @brief Peek a frame from the network (Internal use only)
     * @param receiverID The ID of the node that is receiving the frame
     * @param frame The frame to peek
     * @return True if a frame was peeked successfully, false otherwise
     */
    bool peekFrame(uint32_t receiverID, CANFrame* frame);

    /**
     * @brief Check if a frame is available for a node (Internal use only)
     * @param receiverID The ID of the node that is receiving the frame
     * @return The number of frames available for the node
     */
    uint32_t frameAvailable(uint32_t receiverID);

    /**
     * @brief Check if the network is active (Internal use only)
     * It is active when at least two nodes are connected
     * @return True if the network is active, false otherwise
     */
    bool active();

    void overrideActive(bool forceDisable);

private:
    bool checkNodeID(uint32_t nodeID);
    std::vector<std::list<CANFrame>> network;
    uint32_t nextNodeID = 0;
    bool allowActiveFlag = true;
    OSShim_Mutex* accessMutex = LinuxOSShim().osCreateMutex();
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

    [[nodiscard]] uint32_t getNodeID() const;

    LocalCANNetworkCANShim(LocalCANNetwork* network, uint32_t nodeID);

private:
    LocalCANNetwork* network;
    uint32_t nodeID;
};

#endif // DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H
