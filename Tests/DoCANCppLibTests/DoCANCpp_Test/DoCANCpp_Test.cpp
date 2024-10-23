#include "gtest/gtest.h"
#include "LinuxOSShim.h"
#include "LocalCANNetwork.h"
#include "DoCANCpp.h"


typeof(N_AI::N_SA) N_SA = 123;
uint32_t totalAvailableMemoryForRunners = 1000000;

void N_USData_confirm_cb(N_AI nAi, N_Result nResult, Mtype mType)
{

}

void N_USData_indication_cb(N_AI nAi,uint8_t* messageData, uint32_t messageLength,N_Result nResult, Mtype mtype)
{

}

void N_USData_FF_indication_cb(N_AI nAi, uint32_t expectedMessageLength, Mtype mtype)
{

}

#define INIT_DEFAULT_STRUCTURES \
    LocalCANNetwork network; \
    CANShim* canShim = network.newCANShimConnection(); \
    ASSERT_NE(canShim, nullptr); \
    OSShim* osShim = new LinuxOSShim(); \
    ASSERT_NE(osShim, nullptr);

#define INIT_DEFAULT_STRUCTURES_ALL \
    INIT_DEFAULT_STRUCTURES \
    DoCANCpp doCan(N_SA, totalAvailableMemoryForRunners, N_USData_confirm_cb, N_USData_indication_cb, N_USData_FF_indication_cb, *osShim, *canShim);

void ASSERT_EQ_STMIN(STmin actual, STmin expected)
{
    ASSERT_EQ(actual.unit, expected.unit);
    ASSERT_EQ(actual.value, expected.value);
}

TEST(DoCANCpp, getSTmin_default)
{
    INIT_DEFAULT_STRUCTURES_ALL

    ASSERT_EQ_STMIN(doCan.getSTmin(), DoCANCpp_DefaultSTmin);
}

TEST(DoCANCpp, getSTmin_set_in_constructor)
{
    STmin stMin = {200, us};
    INIT_DEFAULT_STRUCTURES
    DoCANCpp doCan(N_SA, totalAvailableMemoryForRunners, N_USData_confirm_cb, N_USData_indication_cb, N_USData_FF_indication_cb, *osShim, *canShim, 0, stMin);

    ASSERT_EQ_STMIN(doCan.getSTmin(), stMin);
}

TEST(DoCANCpp, setSTmin_valid_getSTmin_from_setSTmin)
{
    STmin stMin = {200, us};
    INIT_DEFAULT_STRUCTURES_ALL

    ASSERT_TRUE(doCan.setSTmin(stMin));
    ASSERT_EQ_STMIN(doCan.getSTmin(), stMin);
}

TEST(DoCANCpp, setSTmin_all)
{
    INIT_DEFAULT_STRUCTURES_ALL
    STmin stMin = {0, ms};
    STmin lastStMin = DoCANCpp_DefaultSTmin;

    for(uint32_t i = 0; i <= UINT16_MAX; i++)
    {
        stMin.value = i;
        if(i > 127)
        {
            ASSERT_FALSE(doCan.setSTmin(stMin));
            ASSERT_EQ_STMIN(doCan.getSTmin(), lastStMin);
        }
        else
        {
            ASSERT_TRUE(doCan.setSTmin(stMin));
            ASSERT_EQ_STMIN(doCan.getSTmin(), stMin);
            lastStMin = stMin;
        }

    }

    stMin.unit = us;
    for(uint32_t i = 100; i <= INT16_MAX; i++)
    {
        stMin.value = i;
        if(i < 100 || i > 900)
        {
            ASSERT_FALSE(doCan.setSTmin(stMin));
            ASSERT_EQ_STMIN(doCan.getSTmin(), lastStMin);
        }
        else
        {
            ASSERT_TRUE(doCan.setSTmin(stMin));
            ASSERT_EQ_STMIN(doCan.getSTmin(), stMin);
            lastStMin = stMin;
        }
    }
}
