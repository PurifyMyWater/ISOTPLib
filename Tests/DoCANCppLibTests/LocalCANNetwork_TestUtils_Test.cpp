#include "gtest/gtest.h"
#include "LocalCANNetwork.h"

#define EXPECT_EQ_FRAMES(frame1, frame2) \
    EXPECT_EQ(frame1.identifier.N_AI, frame2.identifier.N_AI); \
    EXPECT_EQ(frame1.flags, frame2.flags); \
    EXPECT_EQ(frame1.data_length_code, frame2.data_length_code); \
    for (uint32_t i = 0; i < frame1.data_length_code; i++) \
    { \
        EXPECT_EQ(frame1.data[i], frame2.data[i]); \
    }

TEST(LocalCANNetwork, network_newCANShimConnection_test)
{
    LocalCANNetwork network;
    LocalCANNetworkCANShim* can = network.newCANShimConnection();
    ASSERT_NE(can, nullptr);
    EXPECT_EQ(can->getNodeID(), 0);
    delete can;
}

TEST(LocalCANNetwork, network_newCANShimConnection_test_N_nodes)
{
    uint32_t N = 100;
    LocalCANNetwork network;
    for (uint32_t i = 0; i < N; i++)
    {
        LocalCANNetworkCANShim* can = network.newCANShimConnection();
        ASSERT_NE(can, nullptr);
        EXPECT_EQ(can->getNodeID(), i);
        delete can;
    }
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

    EXPECT_EQ(network.active(), false);
    EXPECT_EQ(network.writeFrame(id, &frame), false);
    EXPECT_EQ(network.frameAvailable(id), 0);
    EXPECT_EQ(network.readFrame(id, &frame), false);
    EXPECT_EQ(network.peekFrame(id, &frame), false);
}

TEST(LocalCANNetwork, network_read_write_peek_available_1_node)
{
    LocalCANNetwork network;
    LocalCANNetworkCANShim* can = network.newCANShimConnection();
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

    EXPECT_EQ(network.active(), false);
    EXPECT_EQ(network.writeFrame(id, &frame), false);
    EXPECT_EQ(network.frameAvailable(id), 0);
    EXPECT_EQ(network.readFrame(id, &frame), false);
    EXPECT_EQ(network.peekFrame(id, &frame), false);

    delete can;
}

TEST(LocalCANNetwork, network_read_write_peek_available_1_to_N_node)
{
    uint32_t N = 100;
    LocalCANNetwork network;
    LocalCANNetworkCANShim* wcan = network.newCANShimConnection();
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

    EXPECT_EQ(network.active(), false);

    std::list<LocalCANNetworkCANShim*> rcan;
    for(int i = 1; i<N; i++)
    {
        rcan.push_front(network.newCANShimConnection());
        ASSERT_NE(rcan.front(), nullptr);
    }

    CANFrame fr;

    EXPECT_EQ(network.active(), true);

    EXPECT_EQ(network.writeFrame(writterId, &frame), true);
    EXPECT_EQ(network.frameAvailable(writterId), 0);

    EXPECT_EQ(network.writeFrame(writterId, &frame), true);
    EXPECT_EQ(network.frameAvailable(writterId), 0);

    EXPECT_EQ(network.peekFrame(writterId, &fr), false);
    EXPECT_EQ(network.readFrame(writterId, &fr), false);
    EXPECT_EQ(network.peekFrame(writterId, &fr), false);

    for(auto can : rcan)
    {
        uint32_t id = can->getNodeID();
        EXPECT_EQ(network.frameAvailable(id), 2);
        EXPECT_EQ(network.peekFrame(id, &fr), true);
        EXPECT_EQ_FRAMES(frame, fr);
        EXPECT_EQ(network.frameAvailable(id), 2);

        EXPECT_EQ(network.readFrame(id, &fr), true);
        EXPECT_EQ_FRAMES(frame, fr);
        EXPECT_EQ(network.frameAvailable(id), 1);

        EXPECT_EQ(network.peekFrame(id, &fr), true);
        EXPECT_EQ_FRAMES(frame, fr);
        EXPECT_EQ(network.frameAvailable(id), 1);

        EXPECT_EQ(network.readFrame(id, &fr), true);
        EXPECT_EQ_FRAMES(frame, fr);
        EXPECT_EQ(network.frameAvailable(id), 0);

        EXPECT_EQ(network.peekFrame(id, &fr), false);
        EXPECT_EQ(network.readFrame(id, &fr), false);
        delete can;
    }
}

TEST(LocalCANNetwork, active_test_0_nodes)
{
    LocalCANNetwork network;
    EXPECT_EQ(network.active(), false);
}

TEST(LocalCANNetwork, network_active_test_1_node)
{
    LocalCANNetwork network;
    CANShim* can = network.newCANShimConnection();
    EXPECT_EQ(network.active(), false);
    delete can;
}

TEST(LocalCANNetwork, network_active_test_2_nodes)
{
    LocalCANNetwork network;
    CANShim* can1 = network.newCANShimConnection();
    CANShim* can2 = network.newCANShimConnection();
    EXPECT_EQ(network.active(), true);
    delete can1;
    delete can2;
}

TEST(LocalCANNetwork, network_active_test_N_nodes)
{
    uint32_t N = 100;
    LocalCANNetwork network;
    for (uint32_t i = 0; i < N; i++)
    {
        CANShim* can = network.newCANShimConnection();
        if(i < 2)
        {
            EXPECT_EQ(network.active(), false);
        }
        else
        {
            EXPECT_EQ(network.active(), true);
        }
        delete can;
    }
}
