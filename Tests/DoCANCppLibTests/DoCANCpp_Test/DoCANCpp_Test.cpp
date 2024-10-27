#include "gtest/gtest.h"
#include "ASSERT_MACROS.h"
#include "LinuxOSShim.h"
#include "LocalCANNetwork.h"
#include "DoCANCpp.h"


OSShim* auxOSShim = new LinuxOSShim();
constexpr uint32_t GLOBAL_MESSAGE_TIMEOUT_MS = 2000;

// Fake callbacks used to avoid nullptr exceptions when the tests don't need to check the callbacks.
void N_USData_confirm_dummy_cb(N_AI nAi, N_Result nResult, Mtype mType){}
void N_USData_indication_dummy_cb(N_AI nAi, const uint8_t* messageData, uint32_t messageLength,N_Result nResult, Mtype mtype){}
void N_USData_FF_indication_dummy_cb(N_AI nAi, uint32_t expectedMessageLength, Mtype mtype){}

/// flag to check if the message send has finished
bool send_end_flag;

#define send_loop(DO_CAN_OBJ)do{\
    uint32_t init = auxOSShim->osMillis();\
    while(!send_end_flag && auxOSShim->osMillis() - init < GLOBAL_MESSAGE_TIMEOUT_MS)\
    {\
        DoCANCpp::run_step(&DO_CAN_OBJ);\
    }\
    ASSERT_TRUE(send_end_flag); \
} while(0)


/// flag to check if the reception thread has finished
bool reception_end_flag;

/**
 * @brief Function to run the reception thread
 * This function will run the reception thread until the reception_end_flag is set to true or the GLOBAL_MESSAGE_TIMEOUT_MS is reached
 * This will happen when the reception callback is called or the timeout is reached
 * @warning The test developer should set the reception_end_flag to true in the reception callback or use the provided macros to set the callback
 * @param doCanReception The DoCANCpp object to run the reception thread
 */
void receptionThreadFunction(DoCANCpp* doCanReception)
{
    uint32_t init = auxOSShim->osMillis();
    while(!reception_end_flag && auxOSShim->osMillis() - init < GLOBAL_MESSAGE_TIMEOUT_MS)
    {
        DoCANCpp::run_step(doCanReception);
    }
    ASSERT_TRUE(reception_end_flag);
}

/// Macro to initialize the default structures for the tests
#define INIT_DEFAULT_STRUCTURES(nSA) \
    typeof(N_AI::N_SA) N_SA = nSA; \
    uint32_t totalAvailableMemoryForRunners = 1000000;\
    LocalCANNetwork network; \
    CANShim* canShim = network.newCANShimConnection(); \
    ASSERT_NE(canShim, nullptr); \
    OSShim* osShim = new LinuxOSShim(); \
    ASSERT_NE(osShim, nullptr);

/// Macro to initialize the default structures for the tests and create the default DoCANCpp object
#define INIT_DEFAULT_STRUCTURES_ALL(nSA) \
    INIT_DEFAULT_STRUCTURES(nSA) \
    DoCANCpp doCan(N_SA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShim);

/// Macro to assert the metadata of the message
#define ASSERT_MESSAGE_METADATA(EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE) do { \
    ASSERT_EQ_N_AI(EXPECTED_N_AI, nAi); \
    ASSERT_EQ(messageLength, EXPECTED_MESSAGE_LENGTH); \
    ASSERT_EQ(mtype, EXPECTED_M_TYPE); \
} while(0)

/// Macro to create the N_USData_confirm callback. It will assert the metadata of the message, check if the callback was called (result set in a bool called send_end_flag) and after that you can run your custom code below. Dont forget to close the '}' at the end of your custom code
#define N_USData_confirm_cb(TEST_NAME, EXPECTED_N_AI, EXPECTED_N_RESULT, EXPECTED_M_TYPE) \
    void N_USData_confirm_cb_##TEST_NAME(N_AI nAi, N_Result nResult, Mtype mtype) {       \
    send_end_flag = true;                                                                 \
    N_AI expected_nai = EXPECTED_N_AI;\
    ASSERT_EQ_N_AI(nAi, expected_nai); \
    ASSERT_EQ(nResult, EXPECTED_N_RESULT); \
    ASSERT_EQ(mtype, EXPECTED_M_TYPE);

/// Macro to create the N_USData_confirm callback. It will assert the metadata of the message, check if the callback was called (result set in a bool called N_USData_FF_indication_callback_called_##TEST_NAME) and after that you can run your custom code below. Dont forget to close the '}' at the end of your custom code
#define N_USData_FF_indication_reception_cb(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE) \
    bool N_USData_FF_indication_callback_called_##TEST_NAME = false; \
    void N_USData_FF_indication_reception_cb_##TEST_NAME (N_AI nAi, uint32_t messageLength, Mtype mtype) {      \
    N_USData_FF_indication_callback_called_##TEST_NAME = true;\
    N_AI expected_nai = EXPECTED_N_AI;\
    ASSERT_MESSAGE_METADATA(expected_nai, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE);

/// Macro to create the N_USData_confirm callback. It will assert the metadata of the message and the N_Result, set the reception_end_flag to true and after that you can run your custom code below. Dont forget to close the '}' at the end of your custom code
#define N_USData_indication_reception_cb(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE, EXPECTED_MESSAGE, EXPECTED_N_RESULT) \
    void N_USData_indication_reception_cb_##TEST_NAME (N_AI nAi, const uint8_t* messageData, uint32_t messageLength, N_Result nResult, Mtype mtype) { \
    reception_end_flag = true;                                                                                                     \
    ASSERT_EQ(nResult, EXPECTED_N_RESULT);                                                                                                            \
    N_AI expected_nai = EXPECTED_N_AI;\
    ASSERT_MESSAGE_METADATA(expected_nai, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE);                                                                 \
    for(uint32_t i = 0; i < messageLength; i++) { \
    ASSERT_EQ(messageData[i], EXPECTED_MESSAGE[i]); \
    }

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

// START TEST message_transmission_with_physical_N_TA
const uint8_t message_short_message_transmission_with_physical_N_TA[] = "Hello World!";
const uint32_t messageLength_short_message_transmission_with_physical_N_TA = sizeof(message_short_message_transmission_with_physical_N_TA);
typeof(N_AI::N_SA) N_SA_REC_short_message_transmission_with_physical_N_TA = 210;
typeof(N_AI::N_SA) N_SA_SEND_short_message_transmission_with_physical_N_TA = 123;

N_USData_indication_reception_cb(message_short_transmission_with_physical_N_TA, DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Physical, N_SA_REC_short_message_transmission_with_physical_N_TA, N_SA_SEND_short_message_transmission_with_physical_N_TA), messageLength_short_message_transmission_with_physical_N_TA, Mtype_Diagnostics, message_short_message_transmission_with_physical_N_TA, N_Result::N_OK)

}

N_USData_FF_indication_reception_cb(message_short_transmission_with_physical_N_TA, DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Physical, N_SA_REC_short_message_transmission_with_physical_N_TA, N_SA_SEND_short_message_transmission_with_physical_N_TA), messageLength_short_message_transmission_with_physical_N_TA, Mtype_Diagnostics)

}

N_USData_confirm_cb(message_short_transmission_with_physical_N_TA, DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Physical, N_SA_REC_short_message_transmission_with_physical_N_TA, N_SA_SEND_short_message_transmission_with_physical_N_TA), N_Result::N_OK, Mtype_Diagnostics)

}

TEST(DoCANCpp, message_short_transmission_with_physical_N_TA)
{
    reception_end_flag = false;
    send_end_flag = false;
    INIT_DEFAULT_STRUCTURES(N_SA_SEND_short_message_transmission_with_physical_N_TA)
    CANShim* canShimReception = network.newCANShimConnection();
    ASSERT_NE(canShimReception, nullptr);

    DoCANCpp doCan(N_SA_REC_short_message_transmission_with_physical_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_short_transmission_with_physical_N_TA, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShimReception);
    DoCANCpp doCanReception(N_SA_REC_short_message_transmission_with_physical_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_short_transmission_with_physical_N_TA, N_USData_FF_indication_reception_cb_message_short_transmission_with_physical_N_TA, *osShim, *canShimReception);

    std::thread receptionThread(receptionThreadFunction, &doCanReception);

    doCan.N_USData_request(N_SA_REC_short_message_transmission_with_physical_N_TA, N_TAtype::CAN_CLASSIC_29bit_Physical, (uint8_t*)message_short_message_transmission_with_physical_N_TA, messageLength_short_message_transmission_with_physical_N_TA);

    send_loop(doCan);

    receptionThread.join();
}
// END TEST message_transmission_with_physical_N_TA
