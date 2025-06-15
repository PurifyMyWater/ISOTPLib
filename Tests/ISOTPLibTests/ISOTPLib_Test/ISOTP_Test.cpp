#include "ISOTP.h"

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

TEST(ISOTP, getN_SA)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    ISOTP ISOTP(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, ISOTP_DefaultSTmin);

    EXPECT_EQ(ISOTP.getN_SA(), 1);

    delete canInterface;
}

TEST(ISOTP, getTag)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    ISOTP ISOTP(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, ISOTP_DefaultSTmin, "TestISOTP");

    EXPECT_STREQ(ISOTP.getTag(), "TestISOTP");

    delete canInterface;
}

TEST(ISOTP, AcceptedFunctionalN_TA)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    ISOTP ISOTP(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, ISOTP_DefaultSTmin);

    ISOTP.addAcceptedFunctionalN_TA(2);
    EXPECT_TRUE(ISOTP.hasAcceptedFunctionalN_TA(2));

    EXPECT_TRUE(ISOTP.removeAcceptedFunctionalN_TA(2));
    EXPECT_FALSE(ISOTP.hasAcceptedFunctionalN_TA(2));

    EXPECT_FALSE(ISOTP.removeAcceptedFunctionalN_TA(2));

    delete canInterface;
}

TEST(ISOTP, BlockSize)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    ISOTP ISOTP(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, ISOTP_DefaultSTmin);

    EXPECT_EQ(ISOTP.getBlockSize(), 2);

    EXPECT_TRUE(ISOTP.setBlockSize(3));
    EXPECT_EQ(ISOTP.getBlockSize(), 3);

    delete canInterface;
}

TEST(ISOTP, STmin)
{
    LocalCANNetwork canNetwork;
    CANInterface*   canInterface = canNetwork.newCANInterfaceConnection();

    ISOTP ISOTP(1, 2000, Dummy_N_USData_confirm_cb, Dummy_N_USData_indication_cb, Dummy_N_USData_FF_indication_cb,
                      osInterface, *canInterface, 2, ISOTP_DefaultSTmin);

    EXPECT_EQ(ISOTP.getSTmin().value, ISOTP_DefaultSTmin.value);
    EXPECT_EQ(ISOTP.getSTmin().unit, ISOTP_DefaultSTmin.unit);

    STmin newSTmin = {30, ms};
    EXPECT_TRUE(ISOTP.setSTmin(newSTmin));
    EXPECT_EQ_STMIN(newSTmin, ISOTP.getSTmin());

    EXPECT_TRUE(ISOTP.setSTmin(ISOTP_DefaultSTmin));
    EXPECT_EQ_STMIN(ISOTP_DefaultSTmin, ISOTP.getSTmin());

    STmin invalidSTmin1 = {0, usX100};
    EXPECT_FALSE(ISOTP.setSTmin(invalidSTmin1));
    EXPECT_EQ_STMIN(ISOTP_DefaultSTmin, ISOTP.getSTmin());

    STmin invalidSTmin2 = {10, usX100};
    EXPECT_FALSE(ISOTP.setSTmin(invalidSTmin2));
    EXPECT_EQ_STMIN(ISOTP_DefaultSTmin, ISOTP.getSTmin());

    STmin invalidSTmin3 = {128, ms};
    EXPECT_FALSE(ISOTP.setSTmin(invalidSTmin3));
    EXPECT_EQ_STMIN(ISOTP_DefaultSTmin, ISOTP.getSTmin());

    delete canInterface;
}
