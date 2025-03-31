#include "DoCANCpp.h"

#include <LocalCANNetwork.h>

#include <future>
#include "ASSERT_MACROS.h"
#include "LinuxOSInterface.h"
#include "gtest/gtest.h"

LinuxOSInterface osInterface;
constexpr uint32_t DEFAULT_TIMEOUT = 10000;

volatile bool senderKeepRunning   = true;
volatile bool receiverKeepRunning = true;

// TODO Tests Single Frame, Tests Multiple Frame, Tests with multiple nulls in data, Tests with low memory, Tests with
// different messages to the same N_TA

static uint32_t N_USData_confirm_cb_calls = 0;
void            N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(nResult, N_OK);
    EXPECT_EQ(mtype, Mtype_Diagnostics);
}

static uint32_t N_USData_indication_cb_calls = 0;
void N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength, N_Result nResult, Mtype mtype)
{
    N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ_ARRAY("patata", messageData, 7);
    EXPECT_EQ(messageLength, 7);
    EXPECT_EQ(nResult, N_OK);
    EXPECT_EQ(mtype, Mtype_Diagnostics);

    senderKeepRunning   = false;
    receiverKeepRunning = false;
}

static uint32_t N_USData_FF_indication_cb_calls = 0;
void            N_USData_FF_indication_cb(N_AI nAi, uint32_t messageLength, Mtype mtype)
{
    N_USData_FF_indication_cb_calls++;
    N_AI expectedNAi = {.N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical, .N_TA = 2, .N_SA = 1};
    EXPECT_EQ_N_AI(expectedNAi, nAi);
    EXPECT_EQ(messageLength, 6);
    EXPECT_EQ(mtype, Mtype_Diagnostics);
}

void runStep(DoCANCpp& doCANCpp, const volatile bool& keepRunning, const uint32_t timeout = DEFAULT_TIMEOUT)
{
    uint32_t initialTime = osInterface.osMillis();
    while (keepRunning && osInterface.osMillis() - initialTime < timeout)
    {
        doCANCpp.runStep();
    }
}

void runStepACKQueue(DoCANCpp& doCANCpp, const volatile bool& keepRunning, const uint32_t timeout = DEFAULT_TIMEOUT)
{
    uint32_t initialTime = osInterface.osMillis();
    while (keepRunning && osInterface.osMillis() - initialTime < timeout)
    {
        doCANCpp.canMessageACKQueueRunStep();
    }
}

TEST(DoCANCpp_SystemTests, SimpleSendReceiveTest)
{
    constexpr uint32_t TIMEOUT = 10000;
    senderKeepRunning   = true;
    receiverKeepRunning = true;

    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection();
    CANInterface*   receiverInterface = network.newCANInterfaceConnection();
    DoCANCpp*       senderDoCANCpp    = new DoCANCpp(1, 2000, N_USData_confirm_cb, N_USData_indication_cb,
                                                     N_USData_FF_indication_cb, osInterface, *senderInterface, 2);
    DoCANCpp*       receiverDoCANCpp  = new DoCANCpp(2, 2000, N_USData_confirm_cb, N_USData_indication_cb,
                                                     N_USData_FF_indication_cb, osInterface, *receiverInterface, 2);

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
}
