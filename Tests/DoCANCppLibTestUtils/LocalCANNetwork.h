#ifndef DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H
#define DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H

#include <list>
#include <vector>
#include "CANInterface.h"
#include "LinuxOSInterface.h"

class LocalCANNetworkCANInterface;

/**
 * @brief A local CAN network that can be used to test CANInterface implementations
 * To use it, call newCANInterfaceConnection() to create a new CANInterface connection to the network, and use the CANInterface as you would use normally
 */
class LocalCANNetwork
{
public:
    /**
     * @brief Create a new CANInterface instance connected to the network
     * @return A new CANInterface instance connected to the network
     */
    LocalCANNetworkCANInterface* newCANInterfaceConnection();

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
    bool peekFrame(uint32_t receiverID, CANFrame* frame) const;

    /**
     * @brief Check if a frame is available for a node (Internal use only)
     * @param receiverID The ID of the node that is receiving the frame
     * @return The number of frames available for the node
     */
    [[nodiscard]] uint32_t frameAvailable(uint32_t receiverID) const;

    /**
     * @brief Check if the network is active (Internal use only)
     * It is active when at least two nodes are connected
     * @return True if the network is active, false otherwise
     */
    [[nodiscard]] bool active() const;

    /**
     * @brief Get the ACK result of the last message sent (Internal use only)
     * @return The result of the last ACK or ACK_NONE if no message was transmitted since the last call to this function.
     */
    CANInterface::ACKResult getWriteFrameACK();

    void overrideActive(bool forceDisable);

private:
    CANInterface::ACKResult lastACK = CANInterface::ACK_NONE;
    [[nodiscard]] bool checkNodeID(uint32_t nodeID) const;
    std::vector<std::list<CANFrame>> network;
    uint32_t nextNodeID = 0;
    bool allowActiveFlag = true;
    OSInterface_Mutex* accessMutex = LinuxOSInterface().osCreateMutex();
};

/**
 * @brief A CANInterface implementation that uses a LocalCANNetwork to simulate a CAN bus
 */
class LocalCANNetworkCANInterface : public CANInterface
{
public:
    uint32_t frameAvailable() override;
    bool readFrame(CANFrame* frame) override;
    bool writeFrame(CANFrame* frame) override;
    bool active() override;

    ACKResult getWriteFrameACK() override;

    [[nodiscard]] uint32_t getNodeID() const;

    LocalCANNetworkCANInterface(LocalCANNetwork* network, uint32_t nodeID);

private:
    LocalCANNetwork* network;
    uint32_t nodeID;
};

#endif // DOCANTESTPROJECT_LOCALCANNETWORKMANAGER_H
