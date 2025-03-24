#include "DoCANCpp.h"

#include <LocalCANNetwork.h>

#include <future>
#include "ASSERT_MACROS.h"
#include "LinuxOSInterface.h"
#include "gtest/gtest.h"

LinuxOSInterface osInterface;

// TODO Tests Single Frame, Tests Multiple Frame, Tests with multiple nulls in data, Tests with low memory, Tests with
// different messages to the same N_TA

static uint32_t N_USData_confirm_cb_calls = 0;
void            N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    N_USData_confirm_cb_calls++;

    N_AI expectedNAi = {.N_SA = 1, .N_TA = 2, .N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical};
    ASSERT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ(nResult, N_OK);
    ASSERT_EQ(mtype, Mtype_Diagnostics);
}

static uint32_t N_USData_indication_cb_calls = 0;
void N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength, N_Result nResult, Mtype mtype)
{
    N_USData_indication_cb_calls++;
    N_AI expectedNAi = {.N_SA = 1, .N_TA = 2, .N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical};
    ASSERT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ_ARRAY("patata", messageData, 7);
    ASSERT_EQ(messageLength, 6);
    ASSERT_EQ(nResult, N_OK);
    ASSERT_EQ(mtype, Mtype_Diagnostics);
}

static uint32_t N_USData_FF_indication_cb_calls = 0;
void            N_USData_FF_indication_cb(N_AI nAi, uint32_t messageLength, Mtype mtype)
{
    N_USData_FF_indication_cb_calls++;
    N_AI expectedNAi = {.N_SA = 1, .N_TA = 2, .N_TAtype = N_TATYPE_5_CAN_CLASSIC_29bit_Physical};
    ASSERT_EQ_N_AI(expectedNAi, nAi);
    ASSERT_EQ(messageLength, 6);
    ASSERT_EQ(mtype, Mtype_Diagnostics);
}

void runStep(DoCANCpp& doCANCpp, const bool& keepRunning)
{
    while (keepRunning)
    {
        doCANCpp.runStep();
    }
}

TEST(DoCANCpp_SystemTests, SimpleSendReceiveTest)
{
    LocalCANNetwork network;
    CANInterface*   senderInterface   = network.newCANInterfaceConnection();
    CANInterface*   receiverInterface = network.newCANInterfaceConnection();
    DoCANCpp*       senderDoCANCpp    = new DoCANCpp(1, 2000, N_USData_confirm_cb, N_USData_indication_cb,
                                                     N_USData_FF_indication_cb, osInterface, *senderInterface, 2);
    DoCANCpp*       receiverDoCANCpp  = new DoCANCpp(2, 2000, N_USData_confirm_cb, N_USData_indication_cb,
                                                     N_USData_FF_indication_cb, osInterface, *senderInterface, 2);
    bool senderKeepRunning = true;
    bool receiverKeepRunning = true;

    // Run asyncFunction asynchronously
    std::future<int> result = std::async(std::launch::async, runStep, senderInterface, senderKeepRunning);

    // Do other work while asyncFunction is running
    std::cout << "Doing other work..." << std::endl;

    // Get the result of asyncFunction
    int value = result.get();

    std::cout << "Result: " << value << std::endl;
}
