#include "DoCANCpp.h"

#include <LocalCANNetwork.h>
#include "ASSERT_MACROS.h"
#include "LinuxOSInterface.h"
#include "gtest/gtest.h"

static LinuxOSInterface osInterface;

static uint32_t Dummy_N_USData_confirm_cb_calls = 0;
void            Dummy_N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mtype)
{
    Dummy_N_USData_confirm_cb_calls++;

    OSInterfaceLogInfo("Dummy_N_USData_confirm_cb", "Confirm callback called (%d)", Dummy_N_USData_confirm_cb_calls);
}

static uint32_t Dummy_N_USData_indication_cb_calls = 0;
void Dummy_N_USData_indication_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength, N_Result nResult,
                                  Mtype mtype)
{
    Dummy_N_USData_indication_cb_calls++;

    OSInterfaceLogInfo("Dummy_N_USData_indication_cb", "Indication callback called (%d)",
                       Dummy_N_USData_indication_cb_calls);
}

static uint32_t Dummy_N_USData_FF_indication_cb_calls = 0;
void            Dummy_N_USData_FF_indication_cb(const N_AI nAi, const uint32_t messageLength, const Mtype mtype)
{
    Dummy_N_USData_FF_indication_cb_calls++;

    OSInterfaceLogInfo("Dummy_N_USData_FF_indication_cb", "FF indication callback called (%d)",
                       Dummy_N_USData_FF_indication_cb_calls);
}

TEST(DoCANCpp, getN_SA)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    DoCANCpp doCANCpp(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, DoCANCpp_DefaultSTmin);

    EXPECT_EQ(doCANCpp.getN_SA(), 1);

    delete canInterface;
}

TEST(DoCANCpp, getTag)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    DoCANCpp doCANCpp(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, DoCANCpp_DefaultSTmin, "TestDoCANCpp");

    EXPECT_STREQ(doCANCpp.getTag(), "TestDoCANCpp");

    delete canInterface;
}

TEST(DoCANCpp, AcceptedFunctionalN_TA)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    DoCANCpp doCANCpp(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, DoCANCpp_DefaultSTmin);

    doCANCpp.addAcceptedFunctionalN_TA(2);
    EXPECT_TRUE(doCANCpp.hasAcceptedFunctionalN_TA(2));

    EXPECT_TRUE(doCANCpp.removeAcceptedFunctionalN_TA(2));
    EXPECT_FALSE(doCANCpp.hasAcceptedFunctionalN_TA(2));

    EXPECT_FALSE(doCANCpp.removeAcceptedFunctionalN_TA(2));

    delete canInterface;
}

TEST(DoCANCpp, BlockSize)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    DoCANCpp doCANCpp(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, DoCANCpp_DefaultSTmin);

    EXPECT_EQ(doCANCpp.getBlockSize(), 2);

    EXPECT_TRUE(doCANCpp.setBlockSize(3));
    EXPECT_EQ(doCANCpp.getBlockSize(), 3);

    delete canInterface;
}

TEST(DoCANCpp, STmin)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    DoCANCpp doCANCpp(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, DoCANCpp_DefaultSTmin);

    EXPECT_EQ(doCANCpp.getSTmin().value, DoCANCpp_DefaultSTmin.value);
    EXPECT_EQ(doCANCpp.getSTmin().unit, DoCANCpp_DefaultSTmin.unit);

    STmin newSTmin = {30, ms};
    EXPECT_TRUE(doCANCpp.setSTmin(newSTmin));
    EXPECT_EQ_STMIN(newSTmin, doCANCpp.getSTmin());

    EXPECT_TRUE(doCANCpp.setSTmin(DoCANCpp_DefaultSTmin));
    EXPECT_EQ_STMIN(DoCANCpp_DefaultSTmin, doCANCpp.getSTmin());

    STmin invalidSTmin1 = {0, usX100};
    EXPECT_FALSE(doCANCpp.setSTmin(invalidSTmin1));
    EXPECT_EQ_STMIN(DoCANCpp_DefaultSTmin, doCANCpp.getSTmin());

    STmin invalidSTmin2 = {10, usX100};
    EXPECT_FALSE(doCANCpp.setSTmin(invalidSTmin2));
    EXPECT_EQ_STMIN(DoCANCpp_DefaultSTmin, doCANCpp.getSTmin());

    STmin invalidSTmin3 = {128, ms};
    EXPECT_FALSE(doCANCpp.setSTmin(invalidSTmin3));
    EXPECT_EQ_STMIN(DoCANCpp_DefaultSTmin, doCANCpp.getSTmin());

    delete canInterface;
}
