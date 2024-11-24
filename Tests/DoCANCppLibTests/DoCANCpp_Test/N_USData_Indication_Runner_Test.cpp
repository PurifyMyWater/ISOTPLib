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

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, *can_network.newCANShimConnection());

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

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    const char* testMessageString = "1234567"; // strlen = 7
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, *can_network.newCANShimConnection());

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

    N_AI NAi = DoCANCpp_N_AI_CONFIG(CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin stMin = {10, ms};

    const char* testMessageString = ""; // strlen = 0
    size_t messageLen = strlen(testMessageString);
    const uint8_t* testMessage = reinterpret_cast<const uint8_t*>(testMessageString);

    N_USData_Indication_Runner runner(NAi, availableMemoryMock, blockSize, stMin, linuxOSShim, *can_network.newCANShimConnection());

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
