#include "gtest/gtest.h"
#include "ASSERT_MACROS.h"
#include "N_USData_Request_Runner.h"
#include "DoCANCpp.h"
#include "LinuxOSShim.h"
#include "LocalCANNetwork.h"

static LinuxOSShim linuxOSShim;

TEST(N_USData_Request_Runner, constructor_arguments_set)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = "Message";
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, *can_network.newCANShimConnection());
    ASSERT_EQ(N_USData_Runner::RunnerRequestType, runner.getRunnerType());
    ASSERT_EQ(NOT_STARTED, runner.getResult());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), messageLen);
}

TEST(N_USData_Request_Runner, constructor_destructor_argument_availableMemoryTest)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = "Message";
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    {
        N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen,
                                      linuxOSShim, *can_network.newCANShimConnection());

        int64_t actualMemory;
        ASSERT_TRUE(availableMemoryMock.get(&actualMemory));
        ASSERT_EQ(availableMemoryConst, actualMemory + messageLen);
        ASSERT_TRUE(result);
    }
    int64_t actualMemory;
    ASSERT_TRUE(availableMemoryMock.get(&actualMemory));
    ASSERT_EQ(availableMemoryConst, actualMemory);
}

TEST(N_USData_Request_Runner, constructor_destructor_argument_notAvailableMemoryTest)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 2;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = "Message";
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;


    {
        N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, *can_network.newCANShimConnection());

        int64_t actualMemory;
        ASSERT_TRUE(availableMemoryMock.get(&actualMemory));
        ASSERT_EQ(availableMemoryConst, actualMemory);
        ASSERT_FALSE(result);
    }

    int64_t actualMemory;
    ASSERT_TRUE(availableMemoryMock.get(&actualMemory));
    ASSERT_EQ(availableMemoryConst, actualMemory);
}

TEST(N_USData_Request_Runner, run_step_SF_valid)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "1234567"; // strlen = 7
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;
    CANShim* canShim = can_network.newCANShimConnection();

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, *can_network.newCANShimConnection());

    ASSERT_EQ(N_OK, runner.run_step(nullptr));
    ASSERT_EQ(N_OK, runner.getResult());

    CANFrame receivedFrame;
    ASSERT_TRUE(canShim->readFrame(&receivedFrame));

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(messageLen + 1, receivedFrame.data_length_code); // SF has 1 byte + data for N_PCI_SF
    ASSERT_EQ(N_USData_Runner::SF_CODE, receivedFrame.data[0] >> 4);
    ASSERT_EQ(messageLen, receivedFrame.data[0] & 0x0F);
    ASSERT_EQ(0, memcmp(testMessage, &receivedFrame.data[1], messageLen));
}

TEST(N_USData_Request_Runner, run_step_SF_invalid_big)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = "12345678"; // strlen = 8
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;
    CANShim* canShim = can_network.newCANShimConnection();

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, *can_network.newCANShimConnection());

    ASSERT_EQ(N_ERROR, runner.run_step(nullptr));
    ASSERT_EQ(N_ERROR, runner.getResult());

    CANFrame receivedFrame;
    ASSERT_FALSE(canShim->readFrame(&receivedFrame));
}

TEST(N_USData_Request_Runner, run_step_SF_valid_empty)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = ""; // strlen = 0
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;
    CANShim* canShim = can_network.newCANShimConnection();

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, *can_network.newCANShimConnection());

    ASSERT_EQ(N_OK, runner.run_step(nullptr));
    ASSERT_EQ(N_OK, runner.getResult());

    CANFrame receivedFrame;
    ASSERT_TRUE(canShim->readFrame(&receivedFrame));

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(messageLen + 1, receivedFrame.data_length_code); // SF has 1 byte + data for N_PCI_SF
    ASSERT_EQ(N_USData_Runner::SF_CODE, receivedFrame.data[0] >> 4);
    ASSERT_EQ(messageLen, receivedFrame.data[0] & 0x0F);
    ASSERT_EQ(0, memcmp(testMessage, &receivedFrame.data[1], messageLen));
}
