#include "gtest/gtest.h"
#include "LinuxOSShim.h"
#include "LocalCANNetwork.h"
#include "DoCANCpp.h"


void N_USData_confirm_dummy_cb(N_AI nAi, N_Result nResult, Mtype mType)
{

}

void N_USData_indication_dummy_cb(N_AI nAi,uint8_t* messageData, uint32_t messageLength,N_Result nResult, Mtype mtype)
{

}

void N_USData_FF_indication_dummy_cb(N_AI nAi, uint32_t expectedMessageLength, Mtype mtype)
{

}

#define INIT_DEFAULT_STRUCTURES(nSA) \
    typeof(N_AI::N_SA) N_SA = nSA; \
    uint32_t totalAvailableMemoryForRunners = 1000000;\
    LocalCANNetwork network; \
    CANShim* canShim = network.newCANShimConnection(); \
    ASSERT_NE(canShim, nullptr); \
    OSShim* osShim = new LinuxOSShim(); \
    ASSERT_NE(osShim, nullptr);

#define INIT_DEFAULT_STRUCTURES_ALL(nSA) \
    INIT_DEFAULT_STRUCTURES(nSA) \
    DoCANCpp doCan(N_SA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShim);

void ASSERT_EQ_STMIN(STmin actual, STmin expected)
{
    ASSERT_EQ(actual.unit, expected.unit);
    ASSERT_EQ(actual.value, expected.value);
}

TEST(DoCANCpp, getSTmin_default)
{
    INIT_DEFAULT_STRUCTURES_ALL(123)

    ASSERT_EQ_STMIN(doCan.getSTmin(), DoCANCpp_DefaultSTmin);
}

TEST(DoCANCpp, getSTmin_set_in_constructor)
{
    STmin stMin = {200, us};
    INIT_DEFAULT_STRUCTURES(123)
    DoCANCpp doCan(N_SA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShim, 0, stMin);

    ASSERT_EQ_STMIN(doCan.getSTmin(), stMin);
}

TEST(DoCANCpp, setSTmin_valid_getSTmin_from_setSTmin)
{
    STmin stMin = {200, us};
    INIT_DEFAULT_STRUCTURES_ALL(123)

    ASSERT_TRUE(doCan.setSTmin(stMin));
    ASSERT_EQ_STMIN(doCan.getSTmin(), stMin);
}

TEST(DoCANCpp, setSTmin_all)
{
    INIT_DEFAULT_STRUCTURES_ALL(123)
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
    for(uint32_t i = 0; i <= INT16_MAX; i++)
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

TEST(DoCANCpp, getBlockSize_default)
{
    INIT_DEFAULT_STRUCTURES_ALL(123)

    ASSERT_EQ(doCan.getBlockSize(), DoCANCpp_DefaultBlockSize);
}

TEST(DoCANCpp, getBlockSize_set_in_constructor)
{
    uint8_t blockSize = 200;
    INIT_DEFAULT_STRUCTURES(123)
    DoCANCpp doCan(N_SA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShim, blockSize);

    ASSERT_EQ(doCan.getBlockSize(), blockSize);
}

TEST(DoCANCpp, setBlockSize_valid_getBlockSize_from_setBlockSize)
{
    uint8_t blockSize = 200;
    INIT_DEFAULT_STRUCTURES_ALL(123)

    doCan.setBlockSize(blockSize);
    ASSERT_EQ(doCan.getBlockSize(), blockSize);
}

TEST(DoCANCpp, setBlockSize_all)
{
    INIT_DEFAULT_STRUCTURES_ALL(123)
    uint8_t blockSize = 0;

    for(uint32_t i = 0; i <= UINT8_MAX; i++)
    {
        blockSize = i;
        doCan.setBlockSize(blockSize);
        ASSERT_EQ(doCan.getBlockSize(), blockSize);
    }
}

TEST(DoCANCpp, getN_SA_default)
{
    INIT_DEFAULT_STRUCTURES_ALL(123)

    ASSERT_EQ(doCan.getN_SA(), N_SA);
}

TEST(DoCANCpp, getN_SA_set_in_constructor)
{
    typeof(N_AI::N_SA) nSA = 200;
    INIT_DEFAULT_STRUCTURES(123)
    DoCANCpp doCan(nSA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShim);

    ASSERT_EQ(doCan.getN_SA(), nSA);
}

TEST(DoCANCpp, setN_SA_valid_getN_SA_from_setN_SA)
{
    typeof(N_AI::N_SA) nSA = 200;
    INIT_DEFAULT_STRUCTURES_ALL(123)

    doCan.setN_SA(nSA);
    ASSERT_EQ(doCan.getN_SA(), nSA);
}

TEST(DoCANCpp, setN_SA_all)
{
    INIT_DEFAULT_STRUCTURES_ALL(123)
    typeof(N_AI::N_SA) nSA = 0;
    const typeof(N_AI::N_SA) maxNSA = (typeof(N_AI::N_SA))-1;

    for(uint32_t i = 0; i <= maxNSA; i++)
    {
        nSA = i;
        doCan.setN_SA(nSA);
        ASSERT_EQ(doCan.getN_SA(), nSA);
    }
}

TEST(DoCANCpp, hasAcceptedFunctionalN_TA_default)
{
    INIT_DEFAULT_STRUCTURES_ALL(123)
    const typeof(N_AI::N_TA) maxNTA = (typeof(N_AI::N_TA))-1;

    for(uint32_t i = 0; i <= maxNTA; i++)
    {
        ASSERT_FALSE(doCan.hasAcceptedFunctionalN_TA(i));
    }
}

TEST(DoCANCpp, addAcceptedFunctionalN_TA_hasAcceptedFunctionalN_TA)
{
    const typeof(N_AI::N_TA) nTa = 200;
    INIT_DEFAULT_STRUCTURES_ALL(123)

    doCan.addAcceptedFunctionalN_TA(nTa);
    ASSERT_TRUE(doCan.hasAcceptedFunctionalN_TA(nTa));
}

TEST(DoCANCpp, removeAcceptedFunctionalN_valid_TA_hasAcceptedFunctionalN_TA)
{
    const typeof(N_AI::N_TA) nTa = 200;
    INIT_DEFAULT_STRUCTURES_ALL(123)

    doCan.addAcceptedFunctionalN_TA(nTa);
    ASSERT_TRUE(doCan.hasAcceptedFunctionalN_TA(nTa));

    doCan.removeAcceptedFunctionalN_TA(nTa);
    ASSERT_FALSE(doCan.hasAcceptedFunctionalN_TA(nTa));
}

TEST(DoCANCpp, removeAcceptedFunctionalN_TA_not_added)
{
    const typeof(N_AI::N_TA) nTa = 200;
    INIT_DEFAULT_STRUCTURES_ALL(123)

    ASSERT_FALSE(doCan.removeAcceptedFunctionalN_TA(nTa));
}


