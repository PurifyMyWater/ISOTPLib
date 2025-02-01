#include <DoCANCpp.h>
#include <LocalCANNetwork.h>
#include <N_USData_Indication_Runner.h>
#include "ASSERT_MACROS.h"
#include "gtest/gtest.h"

// TODO Test run_step with a valid SF message, big sf, and empty sf, also test the destructor

static LinuxOSShim linuxOSShim;

TEST(N_USData_Indication_Runner, constructor_getters)
{
    LocalCANNetwork can_network;

    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);

    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, canMessageACKQueue);

    ASSERT_EQ(N_USData_Runner::RunnerIndicationType, runner.getRunnerType());
    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(nullptr, runner.getMessageData());
    ASSERT_EQ(0, runner.getMessageLength());
    ASSERT_EQ(NOT_STARTED, runner.getResult());
    ASSERT_EQ(Mtype_Unknown, runner.getMtype());
}

TEST(N_USData_Indication_Runner, run_step_SF_valid)
{
    LocalCANNetwork can_network;

    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);

    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    const char* testMessageString = "1234567"; // strlen = 7
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, canMessageACKQueue);

    CANFrame sentFrame = NewCANFrameDoCANCpp();
    sentFrame.data[0] = (N_USData_Runner::SF_CODE << 4) | messageLen;
    memcpy(&sentFrame.data[1], testMessage, messageLen);

    ASSERT_EQ(N_OK, runner.run_step(&sentFrame));
    ASSERT_EQ(N_OK, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), messageLen);
}

TEST(N_USData_Indication_Runner, run_step_SF_valid_void)
{
    LocalCANNetwork can_network;

    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);

    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    const char* testMessageString = ""; // strlen = 0
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, canMessageACKQueue);

    CANFrame sentFrame = NewCANFrameDoCANCpp();
    sentFrame.data[0] = (N_USData_Runner::SF_CODE << 4) | messageLen;
    memcpy(&sentFrame.data[1], testMessage, messageLen);

    ASSERT_EQ(N_OK, runner.run_step(&sentFrame));
    ASSERT_EQ(N_OK, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), messageLen);
}

void parseFCFrame(const CANFrame* receivedFrame, N_USData_Runner::FlowStatus fs, uint8_t blockSize, STmin stMin)
{
    ASSERT_NE(nullptr, receivedFrame);
    ASSERT_EQ(CAN_CLASSIC_29bit_Physical, receivedFrame->identifier.N_TAtype);
    ASSERT_EQ(3, receivedFrame->data_length_code);
    ASSERT_EQ(N_USData_Runner::FC_CODE, receivedFrame->data[0] >> 4 & 0x0F);

    auto realFS = static_cast<N_USData_Runner::FlowStatus>(receivedFrame->data[0] & 0b00001111);
    ASSERT_GT(N_USData_Runner::INVALID_FS, realFS);

    uint8_t realBS = receivedFrame->data[1];

    STmin realSTmin;
    if (receivedFrame->data[2] <= 0x7F)
    {
        realSTmin.unit = ms;
        realSTmin.value = receivedFrame->data[2];
    }
    else if (receivedFrame->data[2] >= 0xF1 && receivedFrame->data[2] <= 0xF9)
    {
        realSTmin.unit = usX100;
        realSTmin.value = receivedFrame->data[2] & 0x0F;
    }
    else // Reserved values -> max stMin value
    {
        realSTmin.unit = ms;
        realSTmin.value = 127;
    }

    ASSERT_EQ(fs, realFS);
    ASSERT_EQ(blockSize, realBS);
    ASSERT_EQ(stMin.unit, realSTmin.unit);
    ASSERT_EQ(stMin.value, realSTmin.value);
}

TEST(N_USData_Indication_Runner, run_step_FF_valid)
{
    LocalCANNetwork can_network;

    int64_t availableMemoryConst = 100;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);

    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    const char* testMessageString = "0123456789"; // strlen = 10
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, canMessageACKQueue);

    CANFrame sentFrame = NewCANFrameDoCANCpp();
    sentFrame.data[0] = (N_USData_Runner::FF_CODE << 4) | messageLen >> 8;
    sentFrame.data[1] = messageLen & 0xFF;
    memcpy(&sentFrame.data[2], testMessage, 6);

    CANShim* receiverCanShim = can_network.newCANShimConnection();

    ASSERT_EQ(IN_PROGRESS_FF, runner.run_step(&sentFrame));
    ASSERT_EQ(IN_PROGRESS_FF, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), 6);

    CANFrame receivedFrame;
    ASSERT_TRUE(receiverCanShim->readFrame(&receivedFrame));

    parseFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);
}

TEST(N_USData_Indication_Runner, run_step_FF_big_valid)
{
    LocalCANNetwork can_network;

    int64_t availableMemoryConst = 10000;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);

    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    const char* testMessageString = "0123456789"; // strlen = 10
    size_t messageLen = 5000;
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, canMessageACKQueue);

    CANFrame sentFrame = NewCANFrameDoCANCpp();
    sentFrame.data[0] = (N_USData_Runner::FF_CODE << 4);
    sentFrame.data[1] = 0;
    sentFrame.data[2] = messageLen >> 24 & 0xFF;
    sentFrame.data[3] = messageLen >> 16 & 0xFF;
    sentFrame.data[4] = messageLen >> 8 & 0xFF;
    sentFrame.data[5] = messageLen & 0xFF;
    memcpy(&sentFrame.data[6], testMessage, 2);

    CANShim* receiverCanShim = can_network.newCANShimConnection();

    ASSERT_EQ(IN_PROGRESS_FF, runner.run_step(&sentFrame));
    ASSERT_EQ(IN_PROGRESS_FF, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), 2);

    CANFrame receivedFrame;
    ASSERT_TRUE(receiverCanShim->readFrame(&receivedFrame));

    parseFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);
}

TEST(N_USData_Indication_Runner, run_step_FF_invalid_no_memory)
{
    LocalCANNetwork can_network;

    int64_t availableMemoryConst = 1000;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);

    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    const char* testMessageString = "0123456789"; // strlen = 10
    size_t messageLen = 5000;
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, canMessageACKQueue);

    CANFrame sentFrame = NewCANFrameDoCANCpp();
    sentFrame.data[0] = (N_USData_Runner::FF_CODE << 4);
    sentFrame.data[1] = 0;
    sentFrame.data[2] = messageLen >> 24 & 0xFF;
    sentFrame.data[3] = messageLen >> 16 & 0xFF;
    sentFrame.data[4] = messageLen >> 8 & 0xFF;
    sentFrame.data[5] = messageLen & 0xFF;
    memcpy(&sentFrame.data[6], testMessage, 2);

    ASSERT_EQ(N_ERROR, runner.run_step(&sentFrame));
    ASSERT_EQ(N_ERROR, runner.getResult());
}

TEST(N_USData_Indication_Runner, run_step_FF_nullptr)
{
    LocalCANNetwork can_network;

    int64_t availableMemoryConst = 10000;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSShim);

    CANShim* canShim = can_network.newCANShimConnection();
    CANMessageACKQueue canMessageACKQueue(*canShim);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, canMessageACKQueue);

    ASSERT_EQ(N_ERROR, runner.run_step(nullptr));
    ASSERT_EQ(N_ERROR, runner.getResult());
}