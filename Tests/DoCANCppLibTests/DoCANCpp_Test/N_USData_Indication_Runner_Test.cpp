#include <DoCANCpp.h>
#include <LocalCANNetwork.h>
#include <N_USData_Indication_Runner.h>
#include "ASSERT_MACROS.h"
#include "gtest/gtest.h"

static LinuxOSInterface linuxOSInterface;

constexpr int64_t DEFAULT_AVAILABLE_MEMORY_CONST = 200;

TEST(N_USData_Indication_Runner, constructor_getters)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_6_CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};
    bool    result    = false;
    char    buff[256];
    sprintf(buff, "%s%s", "DoCANCpp_IndicationRunner_", nAiToString(NAi));

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    ASSERT_EQ(result, true);
    ASSERT_STREQ(runner.getTAG(), buff);
    ASSERT_EQ(N_USData_Runner::RunnerIndicationType, runner.getRunnerType());
    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(nullptr, runner.getMessageData());
    ASSERT_EQ(0, runner.getMessageLength());
    ASSERT_EQ(NOT_STARTED, runner.getResult());
    ASSERT_EQ(Mtype_Unknown, runner.getMtype());

    delete canInterface;
}

TEST(N_USData_Indication_Runner, constructor_destructor_argument_availableMemoryTest)
{
    LocalCANNetwork    can_network;
    Atomic_int64_t     availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);
    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);
    N_AI               NAi   = DoCANCpp_N_AI_CONFIG(N_TATYPE_6_CAN_CLASSIC_29bit_Functional, 1, 2);
    int                bs    = 2;
    STmin              stMin = {10, ms};

    {
        bool                       result;
        N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, bs, stMin, linuxOSInterface,
                                          canMessageACKQueue);
        int64_t                    actualMemory;
        ASSERT_TRUE(availableMemoryMock.get(&actualMemory));
        ASSERT_EQ(DEFAULT_AVAILABLE_MEMORY_CONST, actualMemory + N_USDATA_INDICATION_RUNNER_TAG_SIZE);
        ASSERT_TRUE(result);
    }
    int64_t actualMemory;
    ASSERT_TRUE(availableMemoryMock.get(&actualMemory));
    ASSERT_EQ(DEFAULT_AVAILABLE_MEMORY_CONST, actualMemory);

    delete canInterface;
}

TEST(N_USData_Indication_Runner, constructor_destructor_argument_notAvailableMemoryTest)
{
    LocalCANNetwork    can_network;
    int64_t            availableMemoryConst = 2;
    Atomic_int64_t     availableMemoryMock(availableMemoryConst, linuxOSInterface);
    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);
    N_AI               NAi   = DoCANCpp_N_AI_CONFIG(N_TATYPE_6_CAN_CLASSIC_29bit_Functional, 1, 2);
    int                bs    = 2;
    STmin              stMin = {10, ms};

    {
        bool                       result;
        N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, bs, stMin, linuxOSInterface,
                                          canMessageACKQueue);
        int64_t                    actualMemory;
        ASSERT_TRUE(availableMemoryMock.get(&actualMemory));
        ASSERT_EQ(availableMemoryConst, actualMemory);
        ASSERT_FALSE(result);
    }

    int64_t actualMemory;
    ASSERT_TRUE(availableMemoryMock.get(&actualMemory));
    ASSERT_EQ(availableMemoryConst, actualMemory);

    delete canInterface;
}

TEST(N_USData_Indication_Runner, runStep_SF_valid)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_6_CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "1234567"; // strlen = 7
    size_t         messageLen        = strlen(testMessageString);
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::SF_CODE << 4) | messageLen;
    memcpy(&sentFrame.data[1], testMessage, messageLen);

    ASSERT_EQ(N_OK, runner.runStep(&sentFrame));
    ASSERT_EQ(N_OK, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), messageLen);

    delete canInterface;
}

TEST(N_USData_Indication_Runner, runStep_SF_valid_void)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_6_CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = ""; // strlen = 0
    size_t         messageLen        = strlen(testMessageString);
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::SF_CODE << 4) | messageLen;
    memcpy(&sentFrame.data[1], testMessage, messageLen);

    ASSERT_EQ(N_OK, runner.runStep(&sentFrame));
    ASSERT_EQ(N_OK, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), messageLen);
    delete canInterface;
}

TEST(N_USData_Indication_Runner, runStep_SF_Mtype_invalid)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_6_CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "1234567"; // strlen = 7
    size_t         messageLen        = strlen(testMessageString);
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame            = NewCANFrameDoCANCpp();
    sentFrame.identifier          = NAi;
    sentFrame.identifier.N_TAtype = CAN_UNKNOWN; // This should invalidate the Mtype
    sentFrame.data[0]             = (N_USData_Runner::SF_CODE << 4) | messageLen;
    memcpy(&sentFrame.data[1], testMessage, messageLen);

    ASSERT_EQ(N_ERROR, runner.runStep(&sentFrame));
    ASSERT_EQ(N_ERROR, runner.getResult());

    delete canInterface;
}

TEST(N_USData_Indication_Runner, runStep_SF_big_invalid)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_6_CAN_CLASSIC_29bit_Functional, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "1234567890"; // strlen = 10
    size_t         messageLen        = strlen(testMessageString);
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::SF_CODE << 4) | messageLen;
    memcpy(&sentFrame.data[1], testMessage, 7);

    ASSERT_EQ(N_ERROR, runner.runStep(&sentFrame));
    ASSERT_EQ(N_ERROR, runner.getResult());

    delete canInterface;
}

void assertFCFrame(const CANFrame* receivedFrame, N_USData_Runner::FlowStatus fs, uint8_t blockSize, STmin stMin)
{
    ASSERT_NE(nullptr, receivedFrame);
    ASSERT_EQ(N_TATYPE_5_CAN_CLASSIC_29bit_Physical, receivedFrame->identifier.N_TAtype);
    ASSERT_EQ(3, receivedFrame->data_length_code);
    ASSERT_EQ(N_USData_Runner::FC_CODE, receivedFrame->data[0] >> 4 & 0x0F);

    auto realFS = static_cast<N_USData_Runner::FlowStatus>(receivedFrame->data[0] & 0b00001111);
    ASSERT_GT(N_USData_Runner::INVALID_FS, realFS);

    uint8_t realBS = receivedFrame->data[1];

    STmin realSTmin;
    if (receivedFrame->data[2] <= MAX_STMIN_MS_VALUE)
    {
        realSTmin.unit  = ms;
        realSTmin.value = receivedFrame->data[2];
    }
    else if (receivedFrame->data[2] >= MIN_STMIN_US_VALUE && receivedFrame->data[2] <= MAX_STMIN_US_VALUE)
    {
        realSTmin.unit  = usX100;
        realSTmin.value = receivedFrame->data[2] & 0x0F;
    }
    else // Reserved values -> max stMin value
    {
        realSTmin.unit  = ms;
        realSTmin.value = MAX_STMIN_MS_VALUE;
    }

    ASSERT_EQ(fs, realFS);
    ASSERT_EQ(blockSize, realBS);
    ASSERT_EQ(stMin.unit, realSTmin.unit);
    ASSERT_EQ(stMin.value, realSTmin.value);
}

TEST(N_USData_Indication_Runner, runStep_FF_valid)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_5_CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "0123456789"; // strlen = 10
    size_t         messageLen        = strlen(testMessageString);
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::FF_CODE << 4) | messageLen >> 8;
    sentFrame.data[1]    = messageLen & 0xFF;
    memcpy(&sentFrame.data[2], testMessage, 6);

    CANInterface* receiverCanInterface = can_network.newCANInterfaceConnection();

    ASSERT_EQ(IN_PROGRESS_FF, runner.runStep(&sentFrame));
    ASSERT_EQ(IN_PROGRESS_FF, runner.getResult());

    ASSERT_EQ(IN_PROGRESS, runner.runStep(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), 6);

    CANFrame receivedFrame;
    ASSERT_TRUE(receiverCanInterface->readFrame(&receivedFrame));

    ASSERT_EQ(NAi.N_SA, receivedFrame.identifier.N_TA);
    ASSERT_EQ(NAi.N_TA, receivedFrame.identifier.N_SA);

    assertFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);

    delete canInterface;
    delete receiverCanInterface;
}

TEST(N_USData_Indication_Runner, runStep_FF_small)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_5_CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "012345"; // strlen = 6
    size_t         messageLen        = strlen(testMessageString);
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::FF_CODE << 4) | messageLen >> 8;
    sentFrame.data[1]    = messageLen & 0xFF;
    memcpy(&sentFrame.data[2], testMessage, 6);

    CANInterface* receiverCanInterface = can_network.newCANInterfaceConnection();

    ASSERT_EQ(N_ERROR, runner.runStep(&sentFrame));
    ASSERT_EQ(N_ERROR, runner.getResult());

    delete canInterface;
    delete receiverCanInterface;
}

TEST(N_USData_Indication_Runner, runStep_FF_big_valid)
{
    LocalCANNetwork can_network;

    int64_t        availableMemoryConst = 10000;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_5_CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "0123456789"; // strlen = 10
    size_t         messageLen        = 5000;
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::FF_CODE << 4);
    sentFrame.data[1]    = 0;
    sentFrame.data[2]    = messageLen >> 24 & 0xFF;
    sentFrame.data[3]    = messageLen >> 16 & 0xFF;
    sentFrame.data[4]    = messageLen >> 8 & 0xFF;
    sentFrame.data[5]    = messageLen & 0xFF;
    memcpy(&sentFrame.data[6], testMessage, 2);

    CANInterface* receiverCanInterface = can_network.newCANInterfaceConnection();

    ASSERT_EQ(IN_PROGRESS_FF, runner.runStep(&sentFrame));
    ASSERT_EQ(IN_PROGRESS_FF, runner.getResult());

    ASSERT_EQ(IN_PROGRESS, runner.runStep(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), 2);

    CANFrame receivedFrame;
    ASSERT_TRUE(receiverCanInterface->readFrame(&receivedFrame));

    ASSERT_EQ(NAi.N_SA, receivedFrame.identifier.N_TA);
    ASSERT_EQ(NAi.N_TA, receivedFrame.identifier.N_SA);

    assertFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);

    delete canInterface;
    delete receiverCanInterface;
}

TEST(N_USData_Indication_Runner, runStep_FF_invalid_no_memory)
{
    LocalCANNetwork can_network;

    int64_t        availableMemoryConst = 1000;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_5_CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "0123456789"; // strlen = 10
    size_t         messageLen        = 5000;
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::FF_CODE << 4);
    sentFrame.data[1]    = 0;
    sentFrame.data[2]    = messageLen >> 24 & 0xFF;
    sentFrame.data[3]    = messageLen >> 16 & 0xFF;
    sentFrame.data[4]    = messageLen >> 8 & 0xFF;
    sentFrame.data[5]    = messageLen & 0xFF;
    memcpy(&sentFrame.data[6], testMessage, 2);

    ASSERT_EQ(N_ERROR, runner.runStep(&sentFrame));
    ASSERT_EQ(N_ERROR, runner.getResult());

    delete canInterface;
}

TEST(N_USData_Indication_Runner, runStep_FF_nullptr)
{
    LocalCANNetwork can_network;

    int64_t        availableMemoryConst = 10000;
    Atomic_int64_t availableMemoryMock(availableMemoryConst, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_5_CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};
    bool    result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    ASSERT_EQ(N_ERROR, runner.runStep(nullptr));
    ASSERT_EQ(N_ERROR, runner.getResult());

    delete canInterface;
}

TEST(N_USData_Indication_Runner, runStep_CF_valid)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_5_CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "012345678901234567890123456789"; // strlen = 30
    size_t         messageLen        = strlen(testMessageString);
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::FF_CODE << 4) | messageLen >> 8;
    sentFrame.data[1]    = messageLen & 0xFF;
    memcpy(&sentFrame.data[2], testMessage, 6);

    CANInterface* receiverCanInterface = can_network.newCANInterfaceConnection();

    ASSERT_EQ(IN_PROGRESS_FF, runner.runStep(&sentFrame));
    ASSERT_EQ(IN_PROGRESS_FF, runner.getResult());

    ASSERT_EQ(IN_PROGRESS, runner.runStep(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), 6);

    CANFrame receivedFrame;
    ASSERT_TRUE(receiverCanInterface->readFrame(&receivedFrame));
    canMessageACKQueue.runStep(); // Get ACK

    assertFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);

    CANFrame cfFrame         = NewCANFrameDoCANCpp();
    cfFrame.identifier       = NAi;
    cfFrame.data_length_code = 8;
    cfFrame.data[0]          = (N_USData_Runner::CF_CODE << 4) | 1; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[6], 7);

    ASSERT_EQ(IN_PROGRESS, runner.runStep(&cfFrame));

    cfFrame.data[0] = (N_USData_Runner::CF_CODE << 4) | 2; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[13], 7);

    ASSERT_EQ(IN_PROGRESS, runner.runStep(&cfFrame));

    ASSERT_EQ(IN_PROGRESS, runner.runStep(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());

    ASSERT_TRUE(receiverCanInterface->readFrame(&receivedFrame));
    canMessageACKQueue.runStep(); // Get ACK
    assertFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);

    cfFrame.data[0] = (N_USData_Runner::CF_CODE << 4) | 3; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[20], 7);

    ASSERT_EQ(IN_PROGRESS, runner.runStep(&cfFrame));

    cfFrame.data[0] = (N_USData_Runner::CF_CODE << 4) | 4; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[27], 3);

    ASSERT_EQ(N_OK, runner.runStep(&cfFrame));

    delete canInterface;
    delete receiverCanInterface;
}

TEST(N_USData_Indication_Runner, runStep_CF_variable_bs_stmin_valid)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_5_CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 2;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "012345678901234567890123456789"; // strlen = 30
    size_t         messageLen        = strlen(testMessageString);
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::FF_CODE << 4) | messageLen >> 8;
    sentFrame.data[1]    = messageLen & 0xFF;
    memcpy(&sentFrame.data[2], testMessage, 6);

    CANInterface* receiverCanInterface = can_network.newCANInterfaceConnection();

    ASSERT_EQ(IN_PROGRESS_FF, runner.runStep(&sentFrame));
    ASSERT_EQ(IN_PROGRESS_FF, runner.getResult());

    ASSERT_EQ(IN_PROGRESS, runner.runStep(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), 6);

    CANFrame receivedFrame;
    ASSERT_TRUE(receiverCanInterface->readFrame(&receivedFrame));
    canMessageACKQueue.runStep(); // Get ACK

    assertFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);

    CANFrame cfFrame         = NewCANFrameDoCANCpp();
    cfFrame.identifier       = NAi;
    cfFrame.data_length_code = 8;
    cfFrame.data[0]          = (N_USData_Runner::CF_CODE << 4) | 1; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[6], 7);

    stMin.value = 20;
    blockSize   = 1;
    runner.setSTmin(stMin);
    runner.setBlockSize(blockSize);

    ASSERT_EQ(IN_PROGRESS, runner.runStep(&cfFrame));

    ASSERT_FALSE(receiverCanInterface->readFrame(&receivedFrame));

    cfFrame.data[0] = (N_USData_Runner::CF_CODE << 4) | 2; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[13], 7);

    ASSERT_EQ(IN_PROGRESS, runner.runStep(&cfFrame));

    ASSERT_EQ(IN_PROGRESS, runner.runStep(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());

    ASSERT_TRUE(receiverCanInterface->readFrame(&receivedFrame));
    canMessageACKQueue.runStep(); // Get ACK
    assertFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);

    cfFrame.data[0] = (N_USData_Runner::CF_CODE << 4) | 3; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[20], 7);

    ASSERT_EQ(IN_PROGRESS, runner.runStep(&cfFrame));

    ASSERT_EQ(IN_PROGRESS, runner.runStep(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());

    ASSERT_TRUE(receiverCanInterface->readFrame(&receivedFrame));
    canMessageACKQueue.runStep(); // Get ACK
    assertFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);

    cfFrame.data[0] = (N_USData_Runner::CF_CODE << 4) | 4; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[27], 3);

    ASSERT_EQ(N_OK, runner.runStep(&cfFrame));

    delete canInterface;
    delete receiverCanInterface;
}

TEST(N_USData_Indication_Runner, runStep_CF_blockSize0_valid)
{
    LocalCANNetwork can_network;

    Atomic_int64_t availableMemoryMock(DEFAULT_AVAILABLE_MEMORY_CONST, linuxOSInterface);

    CANInterface*      canInterface = can_network.newCANInterfaceConnection();
    CANMessageACKQueue canMessageACKQueue(*canInterface, linuxOSInterface);

    N_AI NAi = DoCANCpp_N_AI_CONFIG(N_TATYPE_5_CAN_CLASSIC_29bit_Physical, 1, 2);

    uint8_t blockSize = 0;
    STmin   stMin     = {10, ms};

    const char*    testMessageString = "012345678901234567890123456789"; // strlen = 30
    size_t         messageLen        = strlen(testMessageString);
    const uint8_t* testMessage       = reinterpret_cast<const uint8_t*>(testMessageString);
    bool           result;

    N_USData_Indication_Runner runner(result, NAi, availableMemoryMock, blockSize, stMin, linuxOSInterface,
                                      canMessageACKQueue);

    CANFrame sentFrame   = NewCANFrameDoCANCpp();
    sentFrame.identifier = NAi;
    sentFrame.data[0]    = (N_USData_Runner::FF_CODE << 4) | messageLen >> 8;
    sentFrame.data[1]    = messageLen & 0xFF;
    memcpy(&sentFrame.data[2], testMessage, 6);

    CANInterface* receiverCanInterface = can_network.newCANInterfaceConnection();

    ASSERT_EQ(IN_PROGRESS_FF, runner.runStep(&sentFrame));
    ASSERT_EQ(IN_PROGRESS_FF, runner.getResult());

    ASSERT_EQ(IN_PROGRESS, runner.runStep(nullptr));
    ASSERT_EQ(IN_PROGRESS, runner.getResult());

    ASSERT_EQ_N_AI(NAi, runner.getN_AI());
    ASSERT_EQ(Mtype_Diagnostics, runner.getMtype());
    ASSERT_EQ(messageLen, runner.getMessageLength());
    ASSERT_EQ_ARRAY(testMessage, runner.getMessageData(), 6);

    CANFrame receivedFrame;
    ASSERT_TRUE(receiverCanInterface->readFrame(&receivedFrame));
    canMessageACKQueue.runStep(); // Get ACK

    assertFCFrame(&receivedFrame, N_USData_Runner::CONTINUE_TO_SEND, blockSize, stMin);

    CANFrame cfFrame         = NewCANFrameDoCANCpp();
    cfFrame.identifier       = NAi;
    cfFrame.data_length_code = 8;
    cfFrame.data[0]          = (N_USData_Runner::CF_CODE << 4) | 1; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[6], 7);

    ASSERT_EQ(IN_PROGRESS, runner.runStep(&cfFrame));

    cfFrame.data[0] = (N_USData_Runner::CF_CODE << 4) | 2; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[13], 7);

    ASSERT_EQ(IN_PROGRESS, runner.runStep(&cfFrame));

    cfFrame.data[0] = (N_USData_Runner::CF_CODE << 4) | 3; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[20], 7);

    ASSERT_EQ(IN_PROGRESS, runner.runStep(&cfFrame));

    cfFrame.data_length_code = 4;
    cfFrame.data[0]          = (N_USData_Runner::CF_CODE << 4) | 4; // sequence number
    memcpy(&cfFrame.data[1], &testMessage[27], 3);

    ASSERT_EQ(N_OK, runner.runStep(&cfFrame));

    delete canInterface;
    delete receiverCanInterface;
}
