#include "N_USData_Request_Runner.h"
#include "ASSERT_MACROS.h"
#include "DoCANCpp.h"
#include "LinuxOSShim.h"
#include "LocalCANNetwork.h"
#include "gtest/gtest.h"

static LinuxOSShim linuxOSShim;

TEST(N_USData_Request_Runner, constructor_arguments_set)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = "Message";
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
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
    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = "Message";
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    {
        N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);

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
    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = "Message";
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    {
        N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);

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
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "1234567"; // strlen = 7
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;
    CANShim* canShim = can_network.newCANShimConnection();

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));

    CANFrame receivedFrame;
    ASSERT_TRUE(canShim->readFrame(&receivedFrame));

    canMessageACKQueue.run_step(); // Get ACK
    ASSERT_EQ(N_OK, runner.run_step(nullptr));
    ASSERT_EQ(N_OK, runner.getResult());

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(messageLen + 1, receivedFrame.data_length_code); // SF has 1 byte + data for N_PCI_SF
    ASSERT_EQ(N_USData_Runner::SF_CODE, receivedFrame.data[0] >> 4);
    ASSERT_EQ(messageLen, receivedFrame.data[0] & 0x0F);
    ASSERT_EQ(0, memcmp(testMessage, &receivedFrame.data[1], messageLen));
}

TEST(N_USData_Request_Runner, run_step_SF_valid_empty)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = ""; // strlen = 0
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;
    CANShim* canShim = can_network.newCANShimConnection();

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));

    CANFrame receivedFrame;
    ASSERT_TRUE(canShim->readFrame(&receivedFrame));

    canMessageACKQueue.run_step(); // Get ACK
    ASSERT_EQ(N_OK, runner.run_step(nullptr));
    ASSERT_EQ(N_OK, runner.getResult());

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(messageLen + 1, receivedFrame.data_length_code); // SF has 1 byte + data for N_PCI_SF
    ASSERT_EQ(N_USData_Runner::SF_CODE, receivedFrame.data[0] >> 4);
    ASSERT_EQ(messageLen, receivedFrame.data[0] & 0x0F);
    ASSERT_EQ(0, memcmp(testMessage, &receivedFrame.data[1], messageLen));
}

TEST(N_USData_Request_Runner, run_step_SF_timeoutAs)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = ""; // strlen = 0
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    can_network.newCANShimConnection();

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));
    linuxOSShim.osSleep(N_USData_Runner::N_As_TIMEOUT_MS + 1);
    ASSERT_EQ(N_TIMEOUT_A, runner.run_step(nullptr));
}

TEST(N_USData_Request_Runner, run_step_SF_unexpectedFrame)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = ""; // strlen = 0
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);

    CANFrame receivedFrame;

    ASSERT_EQ(N_ERROR, runner.run_step(&receivedFrame));
}

TEST(N_USData_Request_Runner, run_step_FF_valid)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "0123456789"; // strlen = 10
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    CANShim* receiverCanShim = can_network.newCANShimConnection();

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());
    CANFrame receivedFrame;

    receiverCanShim->readFrame(&receivedFrame);

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(8, receivedFrame.data_length_code);
    ASSERT_EQ(N_USData_Runner::FF_CODE, receivedFrame.data[0] >> 4);
    uint32_t length = (receivedFrame.data[0] & 0x0F) << 8;
    length = length | receivedFrame.data[1];
    ASSERT_EQ(10, length);
    ASSERT_EQ(0, memcmp(testMessage, &receivedFrame.data[2], 6));
}

TEST(N_USData_Request_Runner, run_step_FF_big_valid)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 10000;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testInitialMessageString = "0123456789"; // strlen = 10

    char* testMessageString = static_cast<char*>(malloc(5001));
    for (int i = 0; i < 500; i++)
    {
        memcpy(&testMessageString[i * 10], testInitialMessageString, 10);
    }
    testMessageString[5000] = '\0';

    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    CANShim* receiverCanShim = can_network.newCANShimConnection();

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());
    CANFrame receivedFrame;

    receiverCanShim->readFrame(&receivedFrame);

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(8, receivedFrame.data_length_code);
    ASSERT_EQ(N_USData_Runner::FF_CODE, receivedFrame.data[0] >> 4);
    uint32_t length = (receivedFrame.data[0] & 0x0F) << 8;
    length = length | receivedFrame.data[1];
    ASSERT_EQ(0, length);
    length = receivedFrame.data[2] << 24 | receivedFrame.data[3] << 16 | receivedFrame.data[4] << 8 |
             receivedFrame.data[5]; // unpack the message length (32 bits) 8 in data[2], 8 in data[3], 8 in data[4] and 8 in data[5]
    ASSERT_EQ(5000, length);
    ASSERT_EQ(0, memcmp(testMessage, &receivedFrame.data[6], 2));
}

TEST(N_USData_Request_Runner, run_step_FF_unexpectedFrame)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "0123456789"; // strlen = 10
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);

    CANFrame receivedFrame = NewCANFrameDoCANCpp();
    ASSERT_EQ(N_ERROR, runner.run_step(&receivedFrame));
    ASSERT_EQ(N_ERROR, runner.getResult());
}

TEST(N_USData_Request_Runner, run_step_FF_wrong_frame_type)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);
    const char* testMessageString = "0123456789"; // strlen = 10
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    ASSERT_FALSE(result);
}

TEST(N_USData_Request_Runner, run_step_First_CF_valid)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "01234567890123456789"; // strlen = 20
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    CANShim* receiverCanShim = can_network.newCANShimConnection();

    runner.run_step(nullptr);
    CANFrame receivedFrame;

    // Indication Runner

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    uint8_t blockSize = 4;
    STmin stMin = {10, ms};

    CANFrame fcFrame = NewCANFrameDoCANCpp();
    fcFrame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    fcFrame.identifier.N_TA = NAi.N_SA;
    fcFrame.identifier.N_SA = NAi.N_TA;

    fcFrame.data[0] = N_USData_Runner::FC_CODE << 4 | N_USData_Runner::FlowStatus::CONTINUE_TO_SEND;
    fcFrame.data[1] = blockSize;
    fcFrame.data[2] = stMin.value;

    fcFrame.data_length_code = 3;

    // Indication Runner

    ASSERT_EQ(IN_PROGRESS, runner.run_step(&fcFrame));
    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));

    receiverCanShim->readFrame(&receivedFrame);

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(8, receivedFrame.data_length_code);
    ASSERT_EQ(N_USData_Runner::CF_CODE, receivedFrame.data[0] >> 4);
    uint8_t sequenceNumber = receivedFrame.data[0] & 0x0F;
    ASSERT_EQ(1, sequenceNumber);
    ASSERT_EQ(0, memcmp(&testMessage[6], &receivedFrame.data[1], 7));
}

TEST(N_USData_Request_Runner, run_step_First_Last_CF_valid)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "0123456789"; // strlen = 10
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    CANShim* receiverCanShim = can_network.newCANShimConnection();

    runner.run_step(nullptr);
    CANFrame receivedFrame;

    // Indication Runner

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    uint8_t blockSize = 4;
    STmin stMin = {10, ms};

    CANFrame fcFrame = NewCANFrameDoCANCpp();
    fcFrame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    fcFrame.identifier.N_TA = NAi.N_SA;
    fcFrame.identifier.N_SA = NAi.N_TA;

    fcFrame.data[0] = N_USData_Runner::FC_CODE << 4 | N_USData_Runner::FlowStatus::CONTINUE_TO_SEND;
    fcFrame.data[1] = blockSize;
    fcFrame.data[2] = stMin.value;

    fcFrame.data_length_code = 3;

    // Indication Runner

    ASSERT_EQ(IN_PROGRESS, runner.run_step(&fcFrame));
    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    ASSERT_EQ(N_OK, runner.run_step(nullptr));

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(5, receivedFrame.data_length_code);
    ASSERT_EQ(N_USData_Runner::CF_CODE, receivedFrame.data[0] >> 4);
    uint8_t sequenceNumber = receivedFrame.data[0] & 0x0F;
    ASSERT_EQ(1, sequenceNumber);
    ASSERT_EQ(0, memcmp(&testMessage[6], &receivedFrame.data[1], 4));
}

TEST(N_USData_Request_Runner, run_step_Intermediate_CF_valid)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "012345678901234567890123456789"; // strlen = 30
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    CANShim* receiverCanShim = can_network.newCANShimConnection();

    runner.run_step(nullptr);
    CANFrame receivedFrame;

    // Indication Runner

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    uint8_t blockSize = 4;
    STmin stMin = {10, ms};

    CANFrame fcFrame = NewCANFrameDoCANCpp();
    fcFrame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    fcFrame.identifier.N_TA = NAi.N_SA;
    fcFrame.identifier.N_SA = NAi.N_TA;

    fcFrame.data[0] = N_USData_Runner::FC_CODE << 4 | N_USData_Runner::FlowStatus::CONTINUE_TO_SEND;
    fcFrame.data[1] = blockSize;
    fcFrame.data[2] = stMin.value;

    fcFrame.data_length_code = 3;

    // Indication Runner

    ASSERT_EQ(IN_PROGRESS, runner.run_step(&fcFrame));
    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(8, receivedFrame.data_length_code);
    ASSERT_EQ(N_USData_Runner::CF_CODE, receivedFrame.data[0] >> 4);
    uint8_t sequenceNumber = receivedFrame.data[0] & 0x0F;
    ASSERT_EQ(2, sequenceNumber);
    ASSERT_EQ(0, memcmp(&testMessage[13], &receivedFrame.data[1], 7));

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));
}

TEST(N_USData_Request_Runner, run_step_Last_CF_valid)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "0123456789012345678901234"; // strlen = 25
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    CANShim* receiverCanShim = can_network.newCANShimConnection();

    runner.run_step(nullptr);
    CANFrame receivedFrame;

    // Indication Runner

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    uint8_t blockSize = 4;
    STmin stMin = {10, ms};

    CANFrame fcFrame = NewCANFrameDoCANCpp();
    fcFrame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    fcFrame.identifier.N_TA = NAi.N_SA;
    fcFrame.identifier.N_SA = NAi.N_TA;

    fcFrame.data[0] = N_USData_Runner::FC_CODE << 4 | N_USData_Runner::FlowStatus::CONTINUE_TO_SEND;
    fcFrame.data[1] = blockSize;
    fcFrame.data[2] = stMin.value;

    fcFrame.data_length_code = 3;

    // Indication Runner

    ASSERT_EQ(IN_PROGRESS, runner.run_step(&fcFrame));
    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    ASSERT_EQ(1, receivedFrame.extd);
    ASSERT_EQ(0, receivedFrame.dlc_non_comp);
    ASSERT_EQ_N_AI(NAi, receivedFrame.identifier);
    ASSERT_EQ(6, receivedFrame.data_length_code);
    ASSERT_EQ(N_USData_Runner::CF_CODE, receivedFrame.data[0] >> 4);
    uint8_t sequenceNumber = receivedFrame.data[0] & 0x0F;
    ASSERT_EQ(3, sequenceNumber);
    ASSERT_EQ(0, memcmp(&testMessage[20], &receivedFrame.data[1], 5));

    ASSERT_EQ(N_OK, runner.run_step(nullptr));
}

TEST(N_USData_Request_Runner, run_step_AnotherFC_valid)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "012345678901234567890123456789"; // strlen = 30
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    CANShim* receiverCanShim = can_network.newCANShimConnection();

    runner.run_step(nullptr);
    CANFrame receivedFrame;

    // Indication Runner

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    uint8_t blockSize = 3;
    STmin stMin = {10, ms};

    CANFrame fcFrame = NewCANFrameDoCANCpp();
    fcFrame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    fcFrame.identifier.N_TA = NAi.N_SA;
    fcFrame.identifier.N_SA = NAi.N_TA;

    fcFrame.data[0] = N_USData_Runner::FC_CODE << 4 | N_USData_Runner::FlowStatus::CONTINUE_TO_SEND;
    fcFrame.data[1] = blockSize;
    fcFrame.data[2] = stMin.value;

    fcFrame.data_length_code = 3;

    // Indication Runner

    ASSERT_EQ(IN_PROGRESS, runner.run_step(&fcFrame));

    for (int32_t i = 0; i<3; i++)
    {
        ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));
        receiverCanShim->readFrame(&receivedFrame);
        canMessageACKQueue.run_step(); // Get ACK
    }

    ASSERT_EQ(IN_PROGRESS, runner.run_step(&fcFrame));

    ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));
    receiverCanShim->readFrame(&receivedFrame);
    canMessageACKQueue.run_step(); // Get ACK
}

TEST(N_USData_Request_Runner, run_step_AnotherFC_NotSent)
{
    LocalCANNetwork can_network;
    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);
    CANShim* canShimRunner = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShimRunner);
    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);
    const char* testMessageString = "012345678901234567890123456789"; // strlen = 30
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);
    bool result;

    N_USData_Request_Runner runner(&result, NAi, availableMemoryMock, Mtype_Diagnostics, testMessage, messageLen, linuxOSShim, canMessageACKQueue);
    CANShim* receiverCanShim = can_network.newCANShimConnection();

    runner.run_step(nullptr);
    CANFrame receivedFrame;

    // Indication Runner

    receiverCanShim->readFrame(&receivedFrame);

    canMessageACKQueue.run_step(); // Get ACK

    uint8_t blockSize = 3;
    STmin stMin = {10, ms};

    CANFrame fcFrame = NewCANFrameDoCANCpp();
    fcFrame.identifier.N_TAtype = CAN_CLASSIC_29bit_Physical;
    fcFrame.identifier.N_TA = NAi.N_SA;
    fcFrame.identifier.N_SA = NAi.N_TA;

    fcFrame.data[0] = N_USData_Runner::FC_CODE << 4 | N_USData_Runner::FlowStatus::CONTINUE_TO_SEND;
    fcFrame.data[1] = blockSize;
    fcFrame.data[2] = stMin.value;

    fcFrame.data_length_code = 3;

    // Indication Runner

    ASSERT_EQ(IN_PROGRESS, runner.run_step(&fcFrame));

    for (int32_t i = 0; i<3; i++)
    {
        ASSERT_EQ(IN_PROGRESS, runner.run_step(nullptr));
        receiverCanShim->readFrame(&receivedFrame);
        canMessageACKQueue.run_step(); // Get ACK
    }

    ASSERT_EQ(N_ERROR, runner.run_step(nullptr));
}
