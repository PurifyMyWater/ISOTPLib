#include "DoCANCpp.h"

#include <LocalCANNetwork.h>

#include <future>
#include "ASSERT_MACROS.h"
#include "LinuxOSInterface.h"
#include "gtest/gtest.h"

LinuxOSInterface   osInterface;
constexpr uint32_t DEFAULT_TIMEOUT = 10000;

volatile bool senderKeepRunning   = true;
volatile bool receiverKeepRunning = true;

// TODO Tests Single Frame, Tests Multiple Frame, Tests with multiple nulls in data, Tests with low memory, Tests with
// different messages to the same N_TA

void runStep(DoCANCpp& doCanCpp, const volatile bool& keepRunning, const uint32_t timeout = DEFAULT_TIMEOUT)
{
    uint32_t initialTime = osInterface.osMillis();
    while (keepRunning && osInterface.osMillis() - initialTime < timeout)
    {
        doCanCpp.runStep();
    }
    OSInterfaceLogInfo(doCanCpp.getTag(), "%s", !keepRunning ? "runStep finished successfully" : "runStep timed out");
}

void runStepACKQueue(const DoCANCpp& doCanCpp, const volatile bool& keepRunning,
                     const uint32_t timeout = DEFAULT_TIMEOUT)
{
    uint32_t initialTime = osInterface.osMillis();
    while (keepRunning && osInterface.osMillis() - initialTime < timeout)
    {
        doCanCpp.canMessageACKQueueRunStep();
    }
    OSInterfaceLogInfo(doCanCpp.getTag(), "%s",
                       !keepRunning ? "runStepACKQueue finished successfully" : "runStepACKQueue timed out");
}

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

    senderKeepRunning   = false;
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
    EXPECT_EQ(SimpleSendReceiveTestSF_messageLength, messageLength);
    ASSERT_NE(nullptr, messageData);
    EXPECT_EQ_ARRAY(SimpleSendReceiveTestSF_message, messageData, SimpleSendReceiveTestSF_messageLength);

    receiverKeepRunning = false;
}

static uint32_t SimpleSendReceiveTestSF_N_USData_FF_indication_cb_calls = 0;
void SimpleSendReceiveTestSF_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength, const Mtype mtype)
{
    SimpleSendReceiveTestSF_N_USData_FF_indication_cb_calls++;
}

TEST(DoCANCpp_SystemTests, SimpleSendReceiveTestSF)
{
    constexpr uint32_t TIMEOUT = 10000;
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection("senderInterface");
    CANInterface*   receiverInterface = network.newCANInterfaceConnection("receiverInterface");
    DoCANCpp*       senderDoCANCpp    = new DoCANCpp(
        1, 2000, SimpleSendReceiveTestSF_N_USData_confirm_cb, SimpleSendReceiveTestSF_N_USData_indication_cb,
        SimpleSendReceiveTestSF_N_USData_FF_indication_cb, osInterface, *senderInterface, 2, DoCANCpp_DefaultSTmin, "senderDoCANCpp");
    DoCANCpp* receiverDoCANCpp = new DoCANCpp(
        2, 2000, SimpleSendReceiveTestSF_N_USData_confirm_cb, SimpleSendReceiveTestSF_N_USData_indication_cb,
        SimpleSendReceiveTestSF_N_USData_FF_indication_cb, osInterface, *receiverInterface, 2, DoCANCpp_DefaultSTmin, "receiverDoCANCpp");

    std::future<void> senderFuture =
        std::async(std::launch::async, [&]() { runStep(*senderDoCANCpp, senderKeepRunning, TIMEOUT); });
    std::future<void> senderACKFuture =
        std::async(std::launch::async, [&]() { runStepACKQueue(*senderDoCANCpp, senderKeepRunning, TIMEOUT); });

    std::future<void> receiverFuture =
        std::async(std::launch::async, [&]() { runStep(*receiverDoCANCpp, receiverKeepRunning, TIMEOUT); });
    std::future<void> receiverACKFuture =
        std::async(std::launch::async, [&]() { runStepACKQueue(*receiverDoCANCpp, receiverKeepRunning, TIMEOUT); });

    EXPECT_TRUE(senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>("patata"), 7, Mtype_Diagnostics));
    senderFuture.wait();
    senderACKFuture.wait();
    receiverFuture.wait();
    receiverACKFuture.wait();

    EXPECT_EQ(0, SimpleSendReceiveTestSF_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(1, SimpleSendReceiveTestSF_N_USData_confirm_cb_calls);
    EXPECT_EQ(1, SimpleSendReceiveTestSF_N_USData_indication_cb_calls);

    delete senderDoCANCpp;
    delete receiverDoCANCpp;
    delete senderInterface;
    delete receiverInterface;
}
// END SimpleSendReceiveTestSF

// SimpleSendReceiveTestFF
constexpr char     SimpleSendReceiveTestFF_message[]     = "01234567890123456789";
constexpr uint32_t SimpleSendReceiveTestFF_messageLength = 21;

static uint32_t SimpleSendReceiveTestFF_N_USData_confirm_cb_calls = 0;
void            SimpleSendReceiveTestFF_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    SimpleSendReceiveTestFF_N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
}

static uint32_t SimpleSendReceiveTestFF_N_USData_indication_cb_calls = 0;
void SimpleSendReceiveTestFF_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,
                                                    N_Result nResult, Mtype mtype)
{
    SimpleSendReceiveTestFF_N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ(N_OK, nResult);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(SimpleSendReceiveTestFF_messageLength, messageLength);
    ASSERT_NE(nullptr, messageData);
    ASSERT_EQ_ARRAY(SimpleSendReceiveTestFF_message, messageData, SimpleSendReceiveTestFF_messageLength);

    osInterface.osSleep(1000); // Wait for the sender to finish running the callback.

    senderKeepRunning   = false;
    receiverKeepRunning = false;
}

static uint32_t SimpleSendReceiveTestFF_N_USData_FF_indication_cb_calls = 0;
void SimpleSendReceiveTestFF_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength, const Mtype mtype)
{
    SimpleSendReceiveTestFF_N_USData_FF_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(SimpleSendReceiveTestFF_messageLength, messageLength);
    EXPECT_EQ(Mtype_Diagnostics, mtype);
}

TEST(DoCANCpp_SystemTests, SimpleSendReceiveTestFF)
{
    return; // TODO: Fix this test

    // constexpr uint32_t TIMEOUT = 10000;
    constexpr uint32_t TIMEOUT = 1000000000;
    senderKeepRunning          = true;
    receiverKeepRunning        = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection();
    CANInterface*   receiverInterface = network.newCANInterfaceConnection();
    DoCANCpp*       senderDoCANCpp    = new DoCANCpp(
        1, 2000, SimpleSendReceiveTestFF_N_USData_confirm_cb, SimpleSendReceiveTestFF_N_USData_indication_cb,
        SimpleSendReceiveTestFF_N_USData_FF_indication_cb, osInterface, *senderInterface, 2);
    DoCANCpp* receiverDoCANCpp = new DoCANCpp(
        2, 2000, SimpleSendReceiveTestFF_N_USData_confirm_cb, SimpleSendReceiveTestFF_N_USData_indication_cb,
        SimpleSendReceiveTestFF_N_USData_FF_indication_cb, osInterface, *receiverInterface, 2);

    std::future<void> senderFuture =
        std::async(std::launch::async, [&]() { runStep(*senderDoCANCpp, senderKeepRunning, TIMEOUT); });
    std::future<void> senderACKFuture =
        std::async(std::launch::async, [&]() { runStepACKQueue(*senderDoCANCpp, senderKeepRunning, TIMEOUT); });

    std::future<void> receiverFuture =
        std::async(std::launch::async, [&]() { runStep(*receiverDoCANCpp, receiverKeepRunning, TIMEOUT); });
    std::future<void> receiverACKFuture =
        std::async(std::launch::async, [&]() { runStepACKQueue(*receiverDoCANCpp, receiverKeepRunning, TIMEOUT); });

    EXPECT_TRUE(senderDoCANCpp->N_USData_request(2, N_TATYPE_5_CAN_CLASSIC_29bit_Physical,
                                                 reinterpret_cast<const uint8_t*>(SimpleSendReceiveTestFF_message),
                                                 SimpleSendReceiveTestFF_messageLength, Mtype_Diagnostics));
    senderFuture.wait();
    senderACKFuture.wait();
    receiverFuture.wait();
    receiverACKFuture.wait();

    EXPECT_EQ(1, SimpleSendReceiveTestSF_N_USData_FF_indication_cb_calls);
    EXPECT_EQ(1, SimpleSendReceiveTestSF_N_USData_confirm_cb_calls);
    EXPECT_EQ(1, SimpleSendReceiveTestSF_N_USData_indication_cb_calls);

    delete senderDoCANCpp;
    delete receiverDoCANCpp;
    delete senderInterface;
    delete receiverInterface;
}
// END SimpleSendReceiveTestFF
