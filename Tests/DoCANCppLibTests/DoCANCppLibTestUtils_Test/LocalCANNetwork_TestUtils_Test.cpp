#include "ASSERT_MACROS.h"
#include "LocalCANNetwork.h"
#include "gtest/gtest.h"

TEST(LocalCANNetwork, network_newCANInterfaceConnection_test)
{
    LocalCANNetwork network;
    LocalCANNetworkCANInterface* can = network.newCANInterfaceConnection();
    ASSERT_NE(can, nullptr);
    ASSERT_EQ(can->getNodeID(), 0);
    delete can;
}

TEST(LocalCANNetwork, network_newCANInterfaceConnection_test_N_nodes)
{
    uint32_t N = 100;
    LocalCANNetwork network;
    for (uint32_t i = 0; i < N; i++)
    {
        LocalCANNetworkCANInterface* can = network.newCANInterfaceConnection();
        ASSERT_NE(can, nullptr);
        ASSERT_EQ(can->getNodeID(), i);
        delete can;
    }
}

TEST(LocalCANNetwork, network_getWriteFrameACK_test_none)
{
    LocalCANNetwork network;
    ASSERT_EQ(network.getWriteFrameACK(), CANInterface::ACK_NONE);
}

TEST(LocalCANNetwork, network_getWriteFrameACK_test_success)
{
    LocalCANNetwork network;
    uint32_t id = 0;
    CANFrame frame;
    frame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    frame.identifier.N_SA = 0;
    frame.identifier.N_TA = 1;
    frame.data_length_code = 8;
    for (uint32_t i = 0; i < 8; i++)
    {
        frame.data[i] = i;
    }

    std::list<LocalCANNetworkCANInterface*> rcan;
    for (int i = 1; i < 3; i++)
    {
        rcan.push_front(network.newCANInterfaceConnection());
        ASSERT_NE(rcan.front(), nullptr);
    }
    ASSERT_EQ(network.active(), true);

    ASSERT_EQ(network.getWriteFrameACK(), CANInterface::ACK_NONE);
    ASSERT_EQ(network.writeFrame(id, &frame), true);
    ASSERT_EQ(network.getWriteFrameACK(), CANInterface::ACK_NONE);
    ASSERT_EQ(network.readFrame(id+1, &frame), true);
    ASSERT_EQ(network.getWriteFrameACK(), CANInterface::ACK_SUCCESS);
    ASSERT_EQ(network.getWriteFrameACK(), CANInterface::ACK_NONE);
}

TEST(LocalCANNetwork, network_read_write_peek_available_0_node)
{
    LocalCANNetwork network;
    uint32_t id = 0;
    CANFrame frame;
    frame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    frame.identifier.N_SA = 0;
    frame.identifier.N_TA = 1;
    frame.data_length_code = 8;
    for (uint32_t i = 0; i < 8; i++)
    {
        frame.data[i] = i;
    }

    ASSERT_EQ(network.active(), false);
    ASSERT_EQ(network.writeFrame(id, &frame), false);
    ASSERT_EQ(network.frameAvailable(id), 0);
    ASSERT_EQ(network.readFrame(id, &frame), false);
    ASSERT_EQ(network.peekFrame(id, &frame), false);
}

TEST(LocalCANNetwork, network_read_write_peek_available_1_node)
{
    LocalCANNetwork network;
    LocalCANNetworkCANInterface* can = network.newCANInterfaceConnection();
    ASSERT_NE(can, nullptr);
    uint32_t id = can->getNodeID();
    CANFrame frame;
    frame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    frame.identifier.N_SA = 0;
    frame.identifier.N_TA = 1;
    frame.data_length_code = 8;
    for (uint32_t i = 0; i < 8; i++)
    {
        frame.data[i] = i;
    }

    ASSERT_EQ(network.active(), false);
    ASSERT_EQ(network.writeFrame(id, &frame), false);
    ASSERT_EQ(network.frameAvailable(id), 0);
    ASSERT_EQ(network.readFrame(id, &frame), false);
    ASSERT_EQ(network.peekFrame(id, &frame), false);

    delete can;
}

TEST(LocalCANNetwork, network_read_write_peek_available_1_to_N_node)
{
    uint32_t N = 100;
    LocalCANNetwork network;
    LocalCANNetworkCANInterface* wcan = network.newCANInterfaceConnection();
    ASSERT_NE(wcan, nullptr);
    uint32_t writterId = wcan->getNodeID();
    CANFrame frame;
    frame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    frame.identifier.N_SA = 0;
    frame.identifier.N_TA = 1;
    frame.data_length_code = 8;
    frame.flags = 1234;
    for (uint32_t i = 0; i < 8; i++)
    {
        frame.data[i] = i;
    }

    ASSERT_EQ(network.active(), false);

    std::list<LocalCANNetworkCANInterface*> rcan;
    for (int i = 1; i < N; i++)
    {
        rcan.push_front(network.newCANInterfaceConnection());
        ASSERT_NE(rcan.front(), nullptr);
    }

    CANFrame fr;

    ASSERT_EQ(network.active(), true);

    ASSERT_EQ(network.writeFrame(writterId, &frame), true);
    ASSERT_EQ(network.frameAvailable(writterId), 0);

    ASSERT_EQ(network.writeFrame(writterId, &frame), true);
    ASSERT_EQ(network.frameAvailable(writterId), 0);

    ASSERT_EQ(network.peekFrame(writterId, &fr), false);
    ASSERT_EQ(network.readFrame(writterId, &fr), false);
    ASSERT_EQ(network.peekFrame(writterId, &fr), false);

    for (auto can: rcan)
    {
        uint32_t id = can->getNodeID();
        ASSERT_EQ(network.frameAvailable(id), 2);
        ASSERT_EQ(network.peekFrame(id, &fr), true);
        ASSERT_EQ_FRAMES(frame, fr);
        ASSERT_EQ(network.frameAvailable(id), 2);

        ASSERT_EQ(network.readFrame(id, &fr), true);
        ASSERT_EQ_FRAMES(frame, fr);
        ASSERT_EQ(network.frameAvailable(id), 1);

        ASSERT_EQ(network.peekFrame(id, &fr), true);
        ASSERT_EQ_FRAMES(frame, fr);
        ASSERT_EQ(network.frameAvailable(id), 1);

        ASSERT_EQ(network.readFrame(id, &fr), true);
        ASSERT_EQ_FRAMES(frame, fr);
        ASSERT_EQ(network.frameAvailable(id), 0);

        ASSERT_EQ(network.peekFrame(id, &fr), false);
        ASSERT_EQ(network.readFrame(id, &fr), false);
        ASSERT_EQ(network.frameAvailable(id), 0);

        delete can;
    }
}

TEST(LocalCANNetwork, active_test_0_nodes)
{
    LocalCANNetwork network;
    ASSERT_EQ(network.active(), false);
}

TEST(LocalCANNetwork, network_active_test_1_node)
{
    LocalCANNetwork network;
    CANInterface* can = network.newCANInterfaceConnection();
    ASSERT_EQ(network.active(), false);
    delete can;
}

TEST(LocalCANNetwork, network_active_test_N_nodes)
{
    uint32_t N = 100;
    LocalCANNetwork network;
    for (uint32_t i = 0; i < N; i++)
    {
        CANInterface* can = network.newCANInterfaceConnection();
        if (i < 1)
        {
            ASSERT_EQ(network.active(), false);
        }
        else
        {
            ASSERT_EQ(network.active(), true);
        }
        delete can;
    }
}

TEST(LocalCANNetwork, CANInterface_active_test_1)
{
    LocalCANNetwork network;
    CANInterface* can = network.newCANInterfaceConnection();
    ASSERT_NE(can, nullptr);
    ASSERT_EQ(can->active(), false);
    delete can;
}

TEST(LocalCANNetwork, CANInterface_active_test_N)
{
    uint32_t N = 100;
    LocalCANNetwork network;
    CANInterface* can1 = network.newCANInterfaceConnection();
    ASSERT_NE(can1, nullptr);
    for (uint32_t i = 0; i < N; i++)
    {
        CANInterface* can = network.newCANInterfaceConnection();
        ASSERT_NE(can, nullptr);
        ASSERT_EQ(can->active(), true);
        ASSERT_EQ(can1->active(), true);
        delete can;
    }
}

TEST(LocalCANNetwork, CANInterface_read_write_peek_available_1_node)
{
    LocalCANNetwork network;
    LocalCANNetworkCANInterface* can = network.newCANInterfaceConnection();
    ASSERT_NE(can, nullptr);
    CANFrame frame;
    frame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    frame.identifier.N_SA = 0;
    frame.identifier.N_TA = 1;
    frame.data_length_code = 8;
    for (uint32_t i = 0; i < 8; i++)
    {
        frame.data[i] = i;
    }

    ASSERT_EQ(can->active(), false);
    ASSERT_EQ(can->writeFrame(&frame), false);
    ASSERT_EQ(can->frameAvailable(), 0);
    ASSERT_EQ(can->readFrame(&frame), false);

    delete can;
}

TEST(LocalCANNetwork, CANInterface_read_write_peek_available_1_to_N_node)
{
    uint32_t N = 100;
    LocalCANNetwork network;
    LocalCANNetworkCANInterface* wcan = network.newCANInterfaceConnection();
    ASSERT_NE(wcan, nullptr);
    CANFrame frame;
    frame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    frame.identifier.N_SA = 0;
    frame.identifier.N_TA = 1;
    frame.data_length_code = 8;
    frame.flags = 1234;
    for (uint32_t i = 0; i < 8; i++)
    {
        frame.data[i] = i;
    }

    ASSERT_EQ(wcan->active(), false);

    std::list<LocalCANNetworkCANInterface*> rcan;
    for (int i = 1; i < N; i++)
    {
        rcan.push_front(network.newCANInterfaceConnection());
        ASSERT_NE(rcan.front(), nullptr);
    }

    CANFrame fr;

    ASSERT_EQ(wcan->active(), true);

    ASSERT_EQ(wcan->writeFrame(&frame), true);
    ASSERT_EQ(wcan->frameAvailable(), 0);

    ASSERT_EQ(wcan->writeFrame(&frame), true);
    ASSERT_EQ(wcan->frameAvailable(), 0);

    ASSERT_EQ(wcan->readFrame(&fr), false);

    for (auto can: rcan)
    {
        uint32_t id = can->getNodeID();
        ASSERT_EQ(can->frameAvailable(), 2);

        ASSERT_EQ(can->readFrame(&fr), true);
        ASSERT_EQ_FRAMES(frame, fr);
        ASSERT_EQ(can->frameAvailable(), 1);

        ASSERT_EQ(can->readFrame(&fr), true);
        ASSERT_EQ_FRAMES(frame, fr);
        ASSERT_EQ(can->frameAvailable(), 0);

        ASSERT_EQ(can->readFrame(&fr), false);
        ASSERT_EQ(can->frameAvailable(), 0);
        delete can;
    }
}
