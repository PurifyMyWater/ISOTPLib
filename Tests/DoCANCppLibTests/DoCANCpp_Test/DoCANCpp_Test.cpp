#include "DoCANCpp.h"

#include <LocalCANNetwork.h>

#include <future>
#include "ASSERT_MACROS.h"
#include "LinuxOSInterface.h"
#include "gtest/gtest.h"

LinuxOSInterface   osInterface;
constexpr uint32_t DEFAULT_TIMEOUT = 10000;

// TODO ~Tests Single Frame, ~Tests Multiple Frame, ~Tests with multiple nulls in data, Tests with low memory, Tests
// with different messages to the same N_TA, Tests with sending and receiving at the same time. ~Test broadcast
// messages, Test DoCANCpp API

volatile bool senderKeepRunning   = true;
volatile bool receiverKeepRunning = true;

// SimpleSendReceiveTestSF
constexpr char     SimpleSendReceiveTestSF_message[]     = "patata";
constexpr uint32_t SimpleSendReceiveTestSF_messageLength = 7;

static uint32_t SimpleSendReceiveTestSF_N_USData_confirm_cb_calls = 0;
void            SimpleSendReceiveTestSF_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    SimpleSendReceiveTestSF_N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);

    OSInterfaceLogInfo("SimpleSendReceiveTestSF_N_USData_confirm_cb", "SenderKeepRunning set to false");
    senderKeepRunning = false;
}

static uint32_t SimpleSendReceiveTestSF_N_USData_indication_cb_calls = 0;
void SimpleSendReceiveTestSF_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,
                                                    N_Result nResult, Mtype mtype)
{
    SimpleSendReceiveTestSF_N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ(SimpleSendReceiveTestSF_messageLength, messageLength);
    ASSERT_NE(nullptr, messageData);
    EXPECT_EQ_ARRAY(SimpleSendReceiveTestSF_message, messageData, SimpleSendReceiveTestSF_messageLength);

    OSInterfaceLogInfo("SimpleSendReceiveTestSF_N_USData_indication_cb", "ReceiverKeepRunning set to false");
    receiverKeepRunning = false;
}

static uint32_t SimpleSendReceiveTestSF_N_USData_FF_indication_cb_calls = 0;
void SimpleSendReceiveTestSF_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength, const Mtype mtype)
{
    SimpleSendReceiveTestSF_N_USData_FF_indication_cb_calls++;
}

TEST(DoCANCpp_SystemTests, SimpleSendReceiveTestSF)
{
    constexpr uint32_t TIMEOUT = 10000; // 10 seconds
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection("senderInterface");
    CANInterface*   receiverInterface = network.newCANInterfaceConnection("receiverInterface");
    DoCANCpp*       senderDoCANCpp =
        new DoCANCpp(1, 2000, SimpleSendReceiveTestSF_N_USData_confirm_cb,
                     SimpleSendReceiveTestSF_N_USData_indication_cb, SimpleSendReceiveTestSF_N_USData_FF_indication_cb,
                     osInterface, *senderInterface, 2, DoCANCpp_DefaultSTmin, "senderDoCANCpp");
    DoCANCpp* receiverDoCANCpp =
        new DoCANCpp(2, 2000, SimpleSendReceiveTestSF_N_USData_confirm_cb,
                     SimpleSendReceiveTestSF_N_USData_indication_cb, SimpleSendReceiveTestSF_N_USData_FF_indication_cb,
                     osInterface, *receiverInterface, 2, DoCANCpp_DefaultSTmin, "receiverDoCANCpp");

    uint32_t initialTime = osInterface.osMillis();
    uint32_t step        = 0;
    while ((senderKeepRunning || receiverKeepRunning) && osInterface.osMillis() - initialTime < TIMEOUT)
    {
        senderDoCANCpp->runStep();
        senderDoCANCpp->canMessageACKQueueRunStep();
        receiverDoCANCpp->runStep();
        receiverDoCANCpp->canMessageACKQueueRunStep();

        if (step == 5)
        {
            EXPECT_TRUE(
                senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>(SimpleSendReceiveTestSF_message),
                                                 SimpleSendReceiveTestSF_messageLength, Mtype_Diagnostics));
        }

        step++;
    }
    uint32_t elapsedTime = osInterface.osMillis() - initialTime;

    EXPECT_EQ(0, SimpleSendReceiveTestSF_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(1, SimpleSendReceiveTestSF_N_USData_confirm_cb_calls);
    EXPECT_EQ(1, SimpleSendReceiveTestSF_N_USData_indication_cb_calls);

    ASSERT_LT(elapsedTime, TIMEOUT) << "Test took too long: " << elapsedTime << " ms, Timeout was: " << TIMEOUT;

    delete senderDoCANCpp;
    delete receiverDoCANCpp;
    delete senderInterface;
    delete receiverInterface;
}
// END SimpleSendReceiveTestSF

// ManySendReceiveTestSF
constexpr char     ManySendReceiveTestSF_message1[]     = "patata";
constexpr uint32_t ManySendReceiveTestSF_messageLength1 = 7;
constexpr char     ManySendReceiveTestSF_message2[]     = "cocida";
constexpr uint32_t ManySendReceiveTestSF_messageLength2 = 7;

static uint32_t ManySendReceiveTestSF_N_USData_confirm_cb_calls = 0;
void            ManySendReceiveTestSF_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    ManySendReceiveTestSF_N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);

    if (ManySendReceiveTestSF_N_USData_confirm_cb_calls == 1)
    {
        OSInterfaceLogInfo("ManySendReceiveTestSF_N_USData_confirm_cb", "First call");
    }

    if (ManySendReceiveTestSF_N_USData_confirm_cb_calls == 2)
    {
        OSInterfaceLogInfo("ManySendReceiveTestSF_N_USData_confirm_cb", "SenderKeepRunning set to false");
        senderKeepRunning = false;
    }
}

static uint32_t ManySendReceiveTestSF_N_USData_indication_cb_calls = 0;
void ManySendReceiveTestSF_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,
                                                  N_Result nResult, Mtype mtype)
{
    ManySendReceiveTestSF_N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    if (ManySendReceiveTestSF_N_USData_indication_cb_calls == 1)
    {
        ASSERT_EQ(ManySendReceiveTestSF_messageLength1, messageLength);
        ASSERT_NE(nullptr, messageData);
        EXPECT_EQ_ARRAY(ManySendReceiveTestSF_message1, messageData, ManySendReceiveTestSF_messageLength1);

        OSInterfaceLogInfo("ManySendReceiveTestSF_N_USData_indication_cb", "First call");
    }
    else if (ManySendReceiveTestSF_N_USData_indication_cb_calls == 2)
    {
        ASSERT_EQ(ManySendReceiveTestSF_messageLength2, messageLength);
        ASSERT_NE(nullptr, messageData);
        EXPECT_EQ_ARRAY(ManySendReceiveTestSF_message2, messageData, ManySendReceiveTestSF_messageLength2);

        OSInterfaceLogInfo("ManySendReceiveTestSF_N_USData_indication_cb", "ReceiverKeepRunning set to false");
        receiverKeepRunning = false;
    }
}

static uint32_t ManySendReceiveTestSF_N_USData_FF_indication_cb_calls = 0;
void ManySendReceiveTestSF_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength, const Mtype mtype)
{
    ManySendReceiveTestSF_N_USData_FF_indication_cb_calls++;
}

TEST(DoCANCpp_SystemTests, ManySendReceiveTestSF)
{
    constexpr uint32_t TIMEOUT = 10000; // 10 seconds
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection("senderInterface");
    CANInterface*   receiverInterface = network.newCANInterfaceConnection("receiverInterface");
    DoCANCpp*       senderDoCANCpp =
        new DoCANCpp(1, 2000, ManySendReceiveTestSF_N_USData_confirm_cb, ManySendReceiveTestSF_N_USData_indication_cb,
                     ManySendReceiveTestSF_N_USData_FF_indication_cb, osInterface, *senderInterface, 2,
                     DoCANCpp_DefaultSTmin, "senderDoCANCpp");
    DoCANCpp* receiverDoCANCpp =
        new DoCANCpp(2, 2000, ManySendReceiveTestSF_N_USData_confirm_cb, ManySendReceiveTestSF_N_USData_indication_cb,
                     ManySendReceiveTestSF_N_USData_FF_indication_cb, osInterface, *receiverInterface, 2,
                     DoCANCpp_DefaultSTmin, "receiverDoCANCpp");

    uint32_t initialTime = osInterface.osMillis();
    uint32_t step        = 0;
    while ((senderKeepRunning || receiverKeepRunning) && osInterface.osMillis() - initialTime < TIMEOUT)
    {
        senderDoCANCpp->runStep();
        senderDoCANCpp->canMessageACKQueueRunStep();
        receiverDoCANCpp->runStep();
        receiverDoCANCpp->canMessageACKQueueRunStep();

        if (step == 5)
        {
            EXPECT_TRUE(
                senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>(ManySendReceiveTestSF_message1),
                                                 ManySendReceiveTestSF_messageLength1, Mtype_Diagnostics));
            EXPECT_TRUE(
                senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>(ManySendReceiveTestSF_message2),
                                                 ManySendReceiveTestSF_messageLength2, Mtype_Diagnostics));
        }

        step++;
    }
    uint32_t elapsedTime = osInterface.osMillis() - initialTime;

    EXPECT_EQ(0, ManySendReceiveTestSF_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(2, ManySendReceiveTestSF_N_USData_confirm_cb_calls);
    EXPECT_EQ(2, ManySendReceiveTestSF_N_USData_indication_cb_calls);

    ASSERT_LT(elapsedTime, TIMEOUT) << "Test took too long: " << elapsedTime << " ms, Timeout was: " << TIMEOUT;

    delete senderDoCANCpp;
    delete receiverDoCANCpp;
    delete senderInterface;
    delete receiverInterface;
}
// END ManySendReceiveTestSF

// SimpleSendReceiveTestBroadcast
constexpr char     SimpleSendReceiveTestBroadcast_message[]     = "patata";
constexpr uint32_t SimpleSendReceiveTestBroadcast_messageLength = 7;

static uint32_t SimpleSendReceiveTestBroadcast_N_USData_confirm_cb_calls = 0;
void            SimpleSendReceiveTestBroadcast_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    SimpleSendReceiveTestBroadcast_N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_TAtype = N_TATYPE_6_CAN_CLASSIC_29bit_Functional, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);

    OSInterfaceLogInfo("SimpleSendReceiveTestBroadcast_N_USData_confirm_cb", "SenderKeepRunning set to false");
    senderKeepRunning = false;
}

static uint32_t SimpleSendReceiveTestBroadcast_N_USData_indication_cb_calls = 0;
void SimpleSendReceiveTestBroadcast_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,
                                                           N_Result nResult, Mtype mtype)
{
    SimpleSendReceiveTestBroadcast_N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_6_CAN_CLASSIC_29bit_Functional, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ(SimpleSendReceiveTestBroadcast_messageLength, messageLength);
    ASSERT_NE(nullptr, messageData);
    EXPECT_EQ_ARRAY(SimpleSendReceiveTestBroadcast_message, messageData, SimpleSendReceiveTestBroadcast_messageLength);

    OSInterfaceLogInfo("SimpleSendReceiveTestBroadcast_N_USData_indication_cb", "ReceiverKeepRunning set to false");
    receiverKeepRunning = false;
}

static uint32_t SimpleSendReceiveTestBroadcast_N_USData_FF_indication_cb_calls = 0;
void            SimpleSendReceiveTestBroadcast_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength,
                                                                         const Mtype mtype)
{
    SimpleSendReceiveTestBroadcast_N_USData_FF_indication_cb_calls++;
}

TEST(DoCANCpp_SystemTests, SimpleSendReceiveTestBroadcast)
{
    constexpr uint32_t TIMEOUT = 10000; // 10 seconds
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface    = network.newCANInterfaceConnection("senderInterface");
    CANInterface*   receiverInterface1 = network.newCANInterfaceConnection("receiverInterface1");
    CANInterface*   receiverInterface2 = network.newCANInterfaceConnection("receiverInterface2");
    DoCANCpp*       senderDoCANCpp     = new DoCANCpp(1, 2000, SimpleSendReceiveTestBroadcast_N_USData_confirm_cb,
                                                      SimpleSendReceiveTestBroadcast_N_USData_indication_cb,
                                                      SimpleSendReceiveTestBroadcast_N_USData_FF_indication_cb, osInterface,
                                                      *senderInterface, 2, DoCANCpp_DefaultSTmin, "senderDoCANCpp");
    DoCANCpp*       receiverDoCANCpp1  = new DoCANCpp(2, 2000, SimpleSendReceiveTestBroadcast_N_USData_confirm_cb,
                                                      SimpleSendReceiveTestBroadcast_N_USData_indication_cb,
                                                      SimpleSendReceiveTestBroadcast_N_USData_FF_indication_cb, osInterface,
                                                      *receiverInterface1, 2, DoCANCpp_DefaultSTmin, "receiverDoCANCpp1");
    DoCANCpp*       receiverDoCANCpp2  = new DoCANCpp(3, 2000, SimpleSendReceiveTestBroadcast_N_USData_confirm_cb,
                                                      SimpleSendReceiveTestBroadcast_N_USData_indication_cb,
                                                      SimpleSendReceiveTestBroadcast_N_USData_FF_indication_cb, osInterface,
                                                      *receiverInterface2, 2, DoCANCpp_DefaultSTmin, "receiverDoCANCpp2");

    uint32_t initialTime = osInterface.osMillis();
    uint32_t step        = 0;

    receiverDoCANCpp1->addAcceptedFunctionalN_TA(2);
    receiverDoCANCpp2->addAcceptedFunctionalN_TA(2);

    while ((senderKeepRunning || receiverKeepRunning) && osInterface.osMillis() - initialTime < TIMEOUT)
    {
        senderDoCANCpp->runStep();
        senderDoCANCpp->canMessageACKQueueRunStep();
        receiverDoCANCpp1->runStep();
        receiverDoCANCpp1->canMessageACKQueueRunStep();
        receiverDoCANCpp2->runStep();
        receiverDoCANCpp2->canMessageACKQueueRunStep();

        if (step == 5)
        {
            EXPECT_TRUE(senderDoCANCpp->N_USData_request(
                2, N_TATYPE_6_CAN_CLASSIC_29bit_Functional,
                reinterpret_cast<const uint8_t*>(SimpleSendReceiveTestBroadcast_message),
                SimpleSendReceiveTestBroadcast_messageLength, Mtype_Diagnostics));
        }

        step++;
    }
    uint32_t elapsedTime = osInterface.osMillis() - initialTime;

    EXPECT_EQ(0, SimpleSendReceiveTestBroadcast_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(1, SimpleSendReceiveTestBroadcast_N_USData_confirm_cb_calls);
    EXPECT_EQ(2, SimpleSendReceiveTestBroadcast_N_USData_indication_cb_calls);

    ASSERT_LT(elapsedTime, TIMEOUT) << "Test took too long: " << elapsedTime << " ms, Timeout was: " << TIMEOUT;

    delete senderDoCANCpp;
    delete receiverDoCANCpp1;
    delete receiverDoCANCpp2;
    delete senderInterface;
    delete receiverInterface1;
    delete receiverInterface2;
}
// END SimpleSendReceiveTestBroadcast

// ManySendReceiveTestBroadcast
constexpr char     ManySendReceiveTestBroadcast_message1[]     = "patata";
constexpr uint32_t ManySendReceiveTestBroadcast_messageLength1 = 7;
constexpr char     ManySendReceiveTestBroadcast_message2[]     = "cocida";
constexpr uint32_t ManySendReceiveTestBroadcast_messageLength2 = 7;

static uint32_t ManySendReceiveTestBroadcast_N_USData_confirm_cb_calls = 0;
void            ManySendReceiveTestBroadcast_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    ManySendReceiveTestBroadcast_N_USData_confirm_cb_calls++;

    if (ManySendReceiveTestBroadcast_N_USData_confirm_cb_calls == 2)
    {
        N_AI expectedNAi = {.N_TAtype = N_TATYPE_6_CAN_CLASSIC_29bit_Functional, .N_TA = 2, .N_SA = 1};
        EXPECT_EQ_N_AI(expectedNAi, nAi);
        EXPECT_EQ(N_OK, nResult);
        EXPECT_EQ(Mtype_Diagnostics, mtype);

        OSInterfaceLogInfo("ManySendReceiveTestBroadcast_N_USData_confirm_cb", "SenderKeepRunning set to false");
        senderKeepRunning = false;
    }

    if (ManySendReceiveTestBroadcast_N_USData_confirm_cb_calls == 1)
    {
        N_AI expectedNAi = {.N_TAtype = N_TATYPE_6_CAN_CLASSIC_29bit_Functional, .N_TA = 3, .N_SA = 1};
        EXPECT_EQ_N_AI(expectedNAi, nAi);
        EXPECT_EQ(N_OK, nResult);
        EXPECT_EQ(Mtype_Diagnostics, mtype);

        OSInterfaceLogInfo("ManySendReceiveTestBroadcast_N_USData_confirm_cb", "First call");
    }
}

static uint32_t ManySendReceiveTestBroadcast_N_USData_indication_cb_calls = 0;
void ManySendReceiveTestBroadcast_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,
                                                         N_Result nResult, Mtype mtype)
{
    ManySendReceiveTestBroadcast_N_USData_indication_cb_calls++;

    if (ManySendReceiveTestBroadcast_N_USData_indication_cb_calls == 2)
    {
        N_AI expectedNAi = {.N_TAtype = N_TATYPE_6_CAN_CLASSIC_29bit_Functional, .N_TA = 2, .N_SA = 1};
        EXPECT_EQ(N_OK, nResult);
        EXPECT_EQ(Mtype_Diagnostics, mtype);
        EXPECT_EQ_N_AI(expectedNAi, nAi);

        ASSERT_EQ(ManySendReceiveTestBroadcast_messageLength1, messageLength);
        ASSERT_NE(nullptr, messageData);
        EXPECT_EQ_ARRAY(ManySendReceiveTestBroadcast_message1, messageData,
                        ManySendReceiveTestBroadcast_messageLength1);

        OSInterfaceLogInfo("ManySendReceiveTestBroadcast_N_USData_indication_cb", "ReceiverKeepRunning set to false");
        receiverKeepRunning = false;
    }
    else if (ManySendReceiveTestBroadcast_N_USData_indication_cb_calls == 1)
    {
        N_AI expectedNAi = {.N_TAtype = N_TATYPE_6_CAN_CLASSIC_29bit_Functional, .N_TA = 3, .N_SA = 1};
        EXPECT_EQ(N_OK, nResult);
        EXPECT_EQ(Mtype_Diagnostics, mtype);
        EXPECT_EQ_N_AI(expectedNAi, nAi);

        ASSERT_EQ(ManySendReceiveTestBroadcast_messageLength2, messageLength);
        ASSERT_NE(nullptr, messageData);
        EXPECT_EQ_ARRAY(ManySendReceiveTestBroadcast_message2, messageData,
                        ManySendReceiveTestBroadcast_messageLength2);

        OSInterfaceLogInfo("ManySendReceiveTestBroadcast_N_USData_indication_cb", "First call");
    }
}

static uint32_t ManySendReceiveTestBroadcast_N_USData_FF_indication_cb_calls = 0;
void            ManySendReceiveTestBroadcast_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength,
                                                                       const Mtype mtype)
{
    ManySendReceiveTestBroadcast_N_USData_FF_indication_cb_calls++;
}

TEST(DoCANCpp_SystemTests, ManySendReceiveTestBroadcast)
{
    constexpr uint32_t TIMEOUT = 10000; // 10 seconds
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface    = network.newCANInterfaceConnection("senderInterface");
    CANInterface*   receiverInterface1 = network.newCANInterfaceConnection("receiverInterface1");
    CANInterface*   receiverInterface2 = network.newCANInterfaceConnection("receiverInterface2");
    DoCANCpp*       senderDoCANCpp     = new DoCANCpp(1, 2000, ManySendReceiveTestBroadcast_N_USData_confirm_cb,
                                                      ManySendReceiveTestBroadcast_N_USData_indication_cb,
                                                      ManySendReceiveTestBroadcast_N_USData_FF_indication_cb, osInterface,
                                                      *senderInterface, 2, DoCANCpp_DefaultSTmin, "senderDoCANCpp");
    DoCANCpp*       receiverDoCANCpp1  = new DoCANCpp(2, 2000, ManySendReceiveTestBroadcast_N_USData_confirm_cb,
                                                      ManySendReceiveTestBroadcast_N_USData_indication_cb,
                                                      ManySendReceiveTestBroadcast_N_USData_FF_indication_cb, osInterface,
                                                      *receiverInterface1, 2, DoCANCpp_DefaultSTmin, "receiverDoCANCpp1");
    DoCANCpp*       receiverDoCANCpp2  = new DoCANCpp(3, 2000, ManySendReceiveTestBroadcast_N_USData_confirm_cb,
                                                      ManySendReceiveTestBroadcast_N_USData_indication_cb,
                                                      SimpleSendReceiveTestBroadcast_N_USData_FF_indication_cb, osInterface,
                                                      *receiverInterface2, 2, DoCANCpp_DefaultSTmin, "receiverDoCANCpp2");

    uint32_t initialTime = osInterface.osMillis();
    uint32_t step        = 0;

    receiverDoCANCpp1->addAcceptedFunctionalN_TA(2);
    receiverDoCANCpp1->addAcceptedFunctionalN_TA(3);

    receiverDoCANCpp2->addAcceptedFunctionalN_TA(2);

    while ((senderKeepRunning || receiverKeepRunning) && osInterface.osMillis() - initialTime < TIMEOUT)
    {
        senderDoCANCpp->runStep();
        senderDoCANCpp->canMessageACKQueueRunStep();
        receiverDoCANCpp1->runStep();
        receiverDoCANCpp1->canMessageACKQueueRunStep();
        receiverDoCANCpp2->runStep();
        receiverDoCANCpp2->canMessageACKQueueRunStep();

        if (step == 5)
        {
            EXPECT_TRUE(senderDoCANCpp->N_USData_request(
                2, N_TATYPE_6_CAN_CLASSIC_29bit_Functional,
                reinterpret_cast<const uint8_t*>(ManySendReceiveTestBroadcast_message1),
                ManySendReceiveTestBroadcast_messageLength1, Mtype_Diagnostics));
            EXPECT_TRUE(senderDoCANCpp->N_USData_request(
                3, N_TATYPE_6_CAN_CLASSIC_29bit_Functional,
                reinterpret_cast<const uint8_t*>(ManySendReceiveTestBroadcast_message2),
                ManySendReceiveTestBroadcast_messageLength2, Mtype_Diagnostics));
        }

        step++;
    }
    uint32_t elapsedTime = osInterface.osMillis() - initialTime;

    EXPECT_EQ(0, ManySendReceiveTestBroadcast_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(2, ManySendReceiveTestBroadcast_N_USData_confirm_cb_calls);
    EXPECT_EQ(3, ManySendReceiveTestBroadcast_N_USData_indication_cb_calls);

    ASSERT_LT(elapsedTime, TIMEOUT) << "Test took too long: " << elapsedTime << " ms, Timeout was: " << TIMEOUT;

    delete senderDoCANCpp;
    delete receiverDoCANCpp1;
    delete receiverDoCANCpp2;
    delete senderInterface;
    delete receiverInterface1;
    delete receiverInterface2;
}
// END ManySendReceiveTestBroadcast

// SimpleSendReceiveTestMF
constexpr char     SimpleSendReceiveTestMF_message[]     = "01234567890123456789";
constexpr uint32_t SimpleSendReceiveTestMF_messageLength = 21;

static uint32_t SimpleSendReceiveTestMF_N_USData_confirm_cb_calls = 0;
void            SimpleSendReceiveTestMF_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    SimpleSendReceiveTestMF_N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);

    OSInterfaceLogInfo("SimpleSendReceiveTestMF_N_USData_confirm_cb", "SenderKeepRunning set to false");
    senderKeepRunning = false;
}

static uint32_t SimpleSendReceiveTestMF_N_USData_indication_cb_calls = 0;
void SimpleSendReceiveTestMF_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,
                                                    N_Result nResult, Mtype mtype)
{
    SimpleSendReceiveTestMF_N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ(SimpleSendReceiveTestMF_messageLength, messageLength);
    ASSERT_NE(nullptr, messageData);
    ASSERT_EQ_ARRAY(SimpleSendReceiveTestMF_message, messageData, SimpleSendReceiveTestMF_messageLength);

    OSInterfaceLogInfo("SimpleSendReceiveTestMF_N_USData_indication_cb", "ReceiverKeepRunning set to false");
    receiverKeepRunning = false;
}

static uint32_t SimpleSendReceiveTestMF_N_USData_FF_indication_cb_calls = 0;
void SimpleSendReceiveTestMF_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength, const Mtype mtype)
{
    SimpleSendReceiveTestMF_N_USData_FF_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ(SimpleSendReceiveTestMF_messageLength, messageLength);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
}

TEST(DoCANCpp_SystemTests, SimpleSendReceiveTestMF)
{
    constexpr uint32_t TIMEOUT = 10000;
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection();
    CANInterface*   receiverInterface = network.newCANInterfaceConnection();
    DoCANCpp*       senderDoCANCpp =
        new DoCANCpp(1, 2000, SimpleSendReceiveTestMF_N_USData_confirm_cb,
                     SimpleSendReceiveTestMF_N_USData_indication_cb, SimpleSendReceiveTestMF_N_USData_FF_indication_cb,
                     osInterface, *senderInterface, 2, DoCANCpp_DefaultSTmin, "senderDoCANCpp");
    DoCANCpp* receiverDoCANCpp =
        new DoCANCpp(2, 2000, SimpleSendReceiveTestMF_N_USData_confirm_cb,
                     SimpleSendReceiveTestMF_N_USData_indication_cb, SimpleSendReceiveTestMF_N_USData_FF_indication_cb,
                     osInterface, *receiverInterface, 2, DoCANCpp_DefaultSTmin, "receiverDoCANCpp");

    uint32_t initialTime = osInterface.osMillis();
    uint32_t step        = 0;
    while ((senderKeepRunning || receiverKeepRunning) && osInterface.osMillis() - initialTime < TIMEOUT)
    {
        senderDoCANCpp->runStep();
        senderDoCANCpp->canMessageACKQueueRunStep();
        receiverDoCANCpp->runStep();
        receiverDoCANCpp->canMessageACKQueueRunStep();

        if (step == 5)
        {
            EXPECT_TRUE(
                senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>(SimpleSendReceiveTestMF_message),
                                                 SimpleSendReceiveTestMF_messageLength, Mtype_Diagnostics));
        }
        step++;
    }
    uint32_t elapsedTime = osInterface.osMillis() - initialTime;

    EXPECT_EQ(1, SimpleSendReceiveTestMF_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(1, SimpleSendReceiveTestMF_N_USData_confirm_cb_calls);
    EXPECT_EQ(1, SimpleSendReceiveTestMF_N_USData_indication_cb_calls);

    ASSERT_LT(elapsedTime, TIMEOUT) << "Test took too long: " << elapsedTime << " ms, Timeout was: " << TIMEOUT;

    delete senderDoCANCpp;
    delete receiverDoCANCpp;
    delete senderInterface;
    delete receiverInterface;
}
// END SimpleSendReceiveTestMF

// ManySendReceiveTestMF
constexpr char     ManySendReceiveTestMF_message1[]     = "01234567890123456789";
constexpr uint32_t ManySendReceiveTestMF_messageLength1 = 21;
constexpr char     ManySendReceiveTestMF_message2[]     = "98765432109876543210";
constexpr uint32_t ManySendReceiveTestMF_messageLength2 = 21;

static uint32_t ManySendReceiveTestMF_N_USData_confirm_cb_calls = 0;
void            ManySendReceiveTestMF_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    ManySendReceiveTestMF_N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);

    if (ManySendReceiveTestMF_N_USData_confirm_cb_calls == 1)
    {
        OSInterfaceLogInfo("ManySendReceiveTestMF_N_USData_confirm_cb", "First call");
    }
    else if (ManySendReceiveTestMF_N_USData_confirm_cb_calls == 2)
    {
        OSInterfaceLogInfo("ManySendReceiveTestMF_N_USData_confirm_cb", "SenderKeepRunning set to false");
        senderKeepRunning = false;
    }
}

static uint32_t ManySendReceiveTestMF_N_USData_indication_cb_calls = 0;
void ManySendReceiveTestMF_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,
                                                  N_Result nResult, Mtype mtype)
{
    ManySendReceiveTestMF_N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};

    if (ManySendReceiveTestMF_N_USData_indication_cb_calls == 1)
    {
        EXPECT_EQ(N_OK, nResult);
        EXPECT_EQ(Mtype_Diagnostics, mtype);
        EXPECT_EQ_N_AI(expectedNAi, nAi);
        ASSERT_EQ(ManySendReceiveTestMF_messageLength1, messageLength);
        ASSERT_NE(nullptr, messageData);
        ASSERT_EQ_ARRAY(ManySendReceiveTestMF_message1, messageData, ManySendReceiveTestMF_messageLength1);

        OSInterfaceLogInfo("ManySendReceiveTestMF_N_USData_indication_cb", "First call");
    }
    else if (ManySendReceiveTestMF_N_USData_indication_cb_calls == 2)
    {
        EXPECT_EQ(N_OK, nResult);
        EXPECT_EQ(Mtype_Diagnostics, mtype);
        EXPECT_EQ_N_AI(expectedNAi, nAi);
        ASSERT_EQ(ManySendReceiveTestMF_messageLength2, messageLength);
        ASSERT_NE(nullptr, messageData);
        ASSERT_EQ_ARRAY(ManySendReceiveTestMF_message2, messageData, ManySendReceiveTestMF_messageLength2);

        OSInterfaceLogInfo("ManySendReceiveTestMF_N_USData_indication_cb", "ReceiverKeepRunning set to false");
        receiverKeepRunning = false;
    }
}

static uint32_t ManySendReceiveTestMF_N_USData_FF_indication_cb_calls = 0;
void ManySendReceiveTestMF_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength, const Mtype mtype)
{
    ManySendReceiveTestMF_N_USData_FF_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    if (ManySendReceiveTestMF_N_USData_FF_indication_cb_calls == 1)
    {
        EXPECT_EQ_N_AI(expectedNAi, nAi);
        ASSERT_EQ(ManySendReceiveTestMF_messageLength1, messageLength);
        EXPECT_EQ(Mtype_Diagnostics, mtype);

        OSInterfaceLogInfo("ManySendReceiveTestMF_N_USData_FF_indication_cb", "First call");
    }
    else if (ManySendReceiveTestMF_N_USData_FF_indication_cb_calls == 2)
    {
        EXPECT_EQ_N_AI(expectedNAi, nAi);
        ASSERT_EQ(ManySendReceiveTestMF_messageLength2, messageLength);
        EXPECT_EQ(Mtype_Diagnostics, mtype);

        OSInterfaceLogInfo("ManySendReceiveTestMF_N_USData_FF_indication_cb", "Second call");
    }
}

TEST(DoCANCpp_SystemTests, ManySendReceiveTestMF)
{
    constexpr uint32_t TIMEOUT = 10000;
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection();
    CANInterface*   receiverInterface = network.newCANInterfaceConnection();
    DoCANCpp*       senderDoCANCpp =
        new DoCANCpp(1, 2000, ManySendReceiveTestMF_N_USData_confirm_cb, ManySendReceiveTestMF_N_USData_indication_cb,
                     ManySendReceiveTestMF_N_USData_FF_indication_cb, osInterface, *senderInterface, 2,
                     DoCANCpp_DefaultSTmin, "senderDoCANCpp");
    DoCANCpp* receiverDoCANCpp =
        new DoCANCpp(2, 2000, ManySendReceiveTestMF_N_USData_confirm_cb, ManySendReceiveTestMF_N_USData_indication_cb,
                     ManySendReceiveTestMF_N_USData_FF_indication_cb, osInterface, *receiverInterface, 2,
                     DoCANCpp_DefaultSTmin, "receiverDoCANCpp");

    uint32_t initialTime = osInterface.osMillis();
    uint32_t step        = 0;
    while ((senderKeepRunning || receiverKeepRunning) && osInterface.osMillis() - initialTime < TIMEOUT)
    {
        senderDoCANCpp->runStep();
        senderDoCANCpp->canMessageACKQueueRunStep();
        receiverDoCANCpp->runStep();
        receiverDoCANCpp->canMessageACKQueueRunStep();

        if (step == 5)
        {
            EXPECT_TRUE(
                senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>(ManySendReceiveTestMF_message1),
                                                 ManySendReceiveTestMF_messageLength1, Mtype_Diagnostics));
            EXPECT_TRUE(
                senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>(ManySendReceiveTestMF_message2),
                                                 ManySendReceiveTestMF_messageLength2, Mtype_Diagnostics));
        }
        step++;
    }
    uint32_t elapsedTime = osInterface.osMillis() - initialTime;

    EXPECT_EQ(2, ManySendReceiveTestMF_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(2, ManySendReceiveTestMF_N_USData_confirm_cb_calls);
    EXPECT_EQ(2, ManySendReceiveTestMF_N_USData_indication_cb_calls);

    ASSERT_LT(elapsedTime, TIMEOUT) << "Test took too long: " << elapsedTime << " ms, Timeout was: " << TIMEOUT;

    delete senderDoCANCpp;
    delete receiverDoCANCpp;
    delete senderInterface;
    delete receiverInterface;
}
// END ManySendReceiveTestMF

// NullCharSendReceiveTestSF
constexpr char     NullCharSendReceiveTestSF_message[]     = "pa\0a\0a";
constexpr uint32_t NullCharSendReceiveTestSF_messageLength = 7;

static uint32_t NullCharSendReceiveTestSF_N_USData_confirm_cb_calls = 0;
void            NullCharSendReceiveTestSF_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    NullCharSendReceiveTestSF_N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);

    OSInterfaceLogInfo("NullCharSendReceiveTestSF_N_USData_confirm_cb", "SenderKeepRunning set to false");
    senderKeepRunning = false;
}

static uint32_t NullCharSendReceiveTestSF_N_USData_indication_cb_calls = 0;
void NullCharSendReceiveTestSF_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,
                                                      N_Result nResult, Mtype mtype)
{
    NullCharSendReceiveTestSF_N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ(NullCharSendReceiveTestSF_messageLength, messageLength);
    ASSERT_NE(nullptr, messageData);
    EXPECT_EQ_ARRAY(NullCharSendReceiveTestSF_message, messageData, NullCharSendReceiveTestSF_messageLength);

    OSInterfaceLogInfo("NullCharSendReceiveTestSF_N_USData_indication_cb", "ReceiverKeepRunning set to false");
    receiverKeepRunning = false;
}

static uint32_t NullCharSendReceiveTestSF_N_USData_FF_indication_cb_calls = 0;
void            NullCharSendReceiveTestSF_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength,
                                                                    const Mtype mtype)
{
    NullCharSendReceiveTestSF_N_USData_FF_indication_cb_calls++;
}

TEST(DoCANCpp_SystemTests, NullCharSendReceiveTestSF)
{
    constexpr uint32_t TIMEOUT = 10000; // 10 seconds
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection("senderInterface");
    CANInterface*   receiverInterface = network.newCANInterfaceConnection("receiverInterface");
    DoCANCpp*       senderDoCANCpp    = new DoCANCpp(1, 2000, NullCharSendReceiveTestSF_N_USData_confirm_cb,
                                                     NullCharSendReceiveTestSF_N_USData_indication_cb,
                                                     NullCharSendReceiveTestSF_N_USData_FF_indication_cb, osInterface,
                                                     *senderInterface, 2, DoCANCpp_DefaultSTmin, "senderDoCANCpp");
    DoCANCpp*       receiverDoCANCpp  = new DoCANCpp(2, 2000, NullCharSendReceiveTestSF_N_USData_confirm_cb,
                                                     NullCharSendReceiveTestSF_N_USData_indication_cb,
                                                     NullCharSendReceiveTestSF_N_USData_FF_indication_cb, osInterface,
                                                     *receiverInterface, 2, DoCANCpp_DefaultSTmin, "receiverDoCANCpp");

    uint32_t initialTime = osInterface.osMillis();
    uint32_t step        = 0;
    while ((senderKeepRunning || receiverKeepRunning) && osInterface.osMillis() - initialTime < TIMEOUT)
    {
        senderDoCANCpp->runStep();
        senderDoCANCpp->canMessageACKQueueRunStep();
        receiverDoCANCpp->runStep();
        receiverDoCANCpp->canMessageACKQueueRunStep();

        if (step == 5)
        {
            EXPECT_TRUE(
                senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>(NullCharSendReceiveTestSF_message),
                                                 NullCharSendReceiveTestSF_messageLength, Mtype_Diagnostics));
        }

        step++;
    }
    uint32_t elapsedTime = osInterface.osMillis() - initialTime;

    EXPECT_EQ(0, NullCharSendReceiveTestSF_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(1, NullCharSendReceiveTestSF_N_USData_confirm_cb_calls);
    EXPECT_EQ(1, NullCharSendReceiveTestSF_N_USData_indication_cb_calls);

    ASSERT_LT(elapsedTime, TIMEOUT) << "Test took too long: " << elapsedTime << " ms, Timeout was: " << TIMEOUT;

    delete senderDoCANCpp;
    delete receiverDoCANCpp;
    delete senderInterface;
    delete receiverInterface;
}
// END NullCharSendReceiveTestSF

// NullCharSendReceiveTestMF
constexpr char     NullCharSendReceiveTestMF_message[]     = "01\0345678901234567\09";
constexpr uint32_t NullCharSendReceiveTestMF_messageLength = 21;

static uint32_t NullCharSendReceiveTestMF_N_USData_confirm_cb_calls = 0;
void            NullCharSendReceiveTestMF_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    NullCharSendReceiveTestMF_N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);

    OSInterfaceLogInfo("NullCharSendReceiveTestMF_N_USData_confirm_cb", "SenderKeepRunning set to false");
    senderKeepRunning = false;
}

static uint32_t NullCharSendReceiveTestMF_N_USData_indication_cb_calls = 0;
void NullCharSendReceiveTestMF_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,
                                                      N_Result nResult, Mtype mtype)
{
    NullCharSendReceiveTestMF_N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ(NullCharSendReceiveTestMF_messageLength, messageLength);
    ASSERT_NE(nullptr, messageData);
    ASSERT_EQ_ARRAY(NullCharSendReceiveTestMF_message, messageData, NullCharSendReceiveTestMF_messageLength);

    OSInterfaceLogInfo("NullCharSendReceiveTestMF_N_USData_indication_cb", "ReceiverKeepRunning set to false");
    receiverKeepRunning = false;
}

static uint32_t NullCharSendReceiveTestMF_N_USData_FF_indication_cb_calls = 0;
void            NullCharSendReceiveTestMF_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength,
                                                                    const Mtype mtype)
{
    NullCharSendReceiveTestMF_N_USData_FF_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ(NullCharSendReceiveTestMF_messageLength, messageLength);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
}

TEST(DoCANCpp_SystemTests, NullCharSendReceiveTestMF)
{
    constexpr uint32_t TIMEOUT = 10000;
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection();
    CANInterface*   receiverInterface = network.newCANInterfaceConnection();
    DoCANCpp*       senderDoCANCpp    = new DoCANCpp(1, 2000, NullCharSendReceiveTestMF_N_USData_confirm_cb,
                                                     NullCharSendReceiveTestMF_N_USData_indication_cb,
                                                     NullCharSendReceiveTestMF_N_USData_FF_indication_cb, osInterface,
                                                     *senderInterface, 2, DoCANCpp_DefaultSTmin, "senderDoCANCpp");
    DoCANCpp*       receiverDoCANCpp  = new DoCANCpp(2, 2000, NullCharSendReceiveTestMF_N_USData_confirm_cb,
                                                     NullCharSendReceiveTestMF_N_USData_indication_cb,
                                                     NullCharSendReceiveTestMF_N_USData_FF_indication_cb, osInterface,
                                                     *receiverInterface, 2, DoCANCpp_DefaultSTmin, "receiverDoCANCpp");

    uint32_t initialTime = osInterface.osMillis();
    uint32_t step        = 0;
    while ((senderKeepRunning || receiverKeepRunning) && osInterface.osMillis() - initialTime < TIMEOUT)
    {
        senderDoCANCpp->runStep();
        senderDoCANCpp->canMessageACKQueueRunStep();
        receiverDoCANCpp->runStep();
        receiverDoCANCpp->canMessageACKQueueRunStep();

        if (step == 5)
        {
            EXPECT_TRUE(
                senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>(NullCharSendReceiveTestMF_message),
                                                 NullCharSendReceiveTestMF_messageLength, Mtype_Diagnostics));
        }
        step++;
    }
    uint32_t elapsedTime = osInterface.osMillis() - initialTime;

    EXPECT_EQ(1, NullCharSendReceiveTestMF_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(1, NullCharSendReceiveTestMF_N_USData_confirm_cb_calls);
    EXPECT_EQ(1, NullCharSendReceiveTestMF_N_USData_indication_cb_calls);

    ASSERT_LT(elapsedTime, TIMEOUT) << "Test took too long: " << elapsedTime << " ms, Timeout was: " << TIMEOUT;

    delete senderDoCANCpp;
    delete receiverDoCANCpp;
    delete senderInterface;
    delete receiverInterface;
}
// END NullCharSendReceiveTestMF
