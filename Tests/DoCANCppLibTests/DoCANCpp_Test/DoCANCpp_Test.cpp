#include "gtest/gtest.h"
#include "ASSERT_MACROS.h"
#include "LinuxOSShim.h"
#include "LocalCANNetwork.h"
#include "DoCANCpp.h"


OSShim* auxOSShim = new LinuxOSShim();
constexpr uint32_t GLOBAL_MESSAGE_TIMEOUT_MS = 2000;

// Fake callbacks used to avoid nullptr exceptions when the tests don't need to check the callbacks.
void N_USData_confirm_dummy_cb([[maybe_unused]] N_AI nAi, [[maybe_unused]] N_Result nResult, [[maybe_unused]] Mtype mType){}
void N_USData_indication_dummy_cb([[maybe_unused]] N_AI nAi, [[maybe_unused]] const uint8_t* messageData, [[maybe_unused]] uint32_t messageLength, [[maybe_unused]] N_Result nResult, [[maybe_unused]] Mtype mtype){}
void N_USData_FF_indication_dummy_cb([[maybe_unused]] N_AI nAi, [[maybe_unused]] uint32_t expectedMessageLength, [[maybe_unused]] Mtype mtype){}

/// flag to check if the message send has finished
volatile bool send_end_flag;

#define send_loop(DO_CAN_OBJ, expected_send_end_flag)do{\
    uint32_t init = auxOSShim->osMillis();\
    while(!send_end_flag && auxOSShim->osMillis() - init < GLOBAL_MESSAGE_TIMEOUT_MS)\
    {\
        DoCANCpp::run_step(&DO_CAN_OBJ);\
    }\
    ASSERT_EQ(send_end_flag, expected_send_end_flag); \
} while(0)


/// flag to check if the reception thread has finished
volatile bool reception_end_flag;

/**
 * @brief Function to run the reception thread
 * This function will run the reception thread until the reception_end_flag is set to true or the GLOBAL_MESSAGE_TIMEOUT_MS is reached
 * This will happen when the reception callback is called or the timeout is reached
 * @warning The test developer should set the reception_end_flag to true in the reception callback or use the provided macros to set the callback
 * @param doCanReception The DoCANCpp object to run the reception thread
 */
void receptionThreadFunction(DoCANCpp* doCanReception, bool expected_reception_end_flag)
{
    uint32_t init = auxOSShim->osMillis();
    while(!reception_end_flag && auxOSShim->osMillis() - init < GLOBAL_MESSAGE_TIMEOUT_MS)
    {
        DoCANCpp::run_step(doCanReception);
    }
    ASSERT_EQ(reception_end_flag, expected_reception_end_flag);
}

/// flag to check if the reception thread has finished
volatile bool reception_end_flag_2;

/**
 * @brief Function to run the reception thread
 * This function will run the reception thread until the reception_end_flag is set to true or the GLOBAL_MESSAGE_TIMEOUT_MS is reached
 * This will happen when the reception callback is called or the timeout is reached
 * @warning The test developer should set the reception_end_flag to true in the reception callback or use the provided macros to set the callback
 * @param doCanReception The DoCANCpp object to run the reception thread
 */
void receptionThreadFunction_2(DoCANCpp* doCanReception, bool expected_reception_end_flag)
{
    uint32_t init = auxOSShim->osMillis();
    while(!reception_end_flag && auxOSShim->osMillis() - init < GLOBAL_MESSAGE_TIMEOUT_MS)
    {
        DoCANCpp::run_step(doCanReception);
    }
    ASSERT_EQ(reception_end_flag, expected_reception_end_flag);
}

/// Macro to initialize the default structures for the tests
#define INIT_DEFAULT_STRUCTURES(nSA) \
    reception_end_flag = false; \
    reception_end_flag_2 = false; \
    send_end_flag = false; \
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
#define N_USData_confirm_cb_internal(TEST_NAME, EXPECTED_N_AI, EXPECTED_N_RESULT, EXPECTED_M_TYPE, SEND_END_FLAG_NAME) \
    void N_USData_confirm_cb_##TEST_NAME(N_AI nAi, N_Result nResult, Mtype mtype) {       \
    send_end_flag = true;                                                                 \
    N_AI expected_nai = EXPECTED_N_AI;\
    ASSERT_EQ_N_AI(nAi, expected_nai); \
    ASSERT_EQ(nResult, EXPECTED_N_RESULT); \
    ASSERT_EQ(mtype, EXPECTED_M_TYPE);

/// Macro to create the N_USData_confirm callback. It will assert the metadata of the message, check if the callback was called (result set in a bool called N_USData_FF_indication_callback_called_##TEST_NAME) and after that you can run your custom code below. Dont forget to close the '}' at the end of your custom code
#define N_USData_FF_indication_reception_cb_internal(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE) \
    volatile bool N_USData_FF_indication_callback_called_##TEST_NAME = false; \
    void N_USData_FF_indication_reception_cb_##TEST_NAME (N_AI nAi, uint32_t messageLength, Mtype mtype) {      \
    N_USData_FF_indication_callback_called_##TEST_NAME = true;\
    N_AI expected_nai = EXPECTED_N_AI;\
    ASSERT_MESSAGE_METADATA(expected_nai, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE);

/// Macro to create the N_USData_confirm callback. It will assert the metadata of the message and the N_Result, set the reception_end_flag to true and after that you can run your custom code below. Dont forget to close the '}' at the end of your custom code
#define N_USData_indication_reception_cb_internal(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE, EXPECTED_MESSAGE, EXPECTED_N_RESULT, RECEPTION_END_FLAG_NAME) \
    void N_USData_indication_reception_cb_##TEST_NAME (N_AI nAi, const uint8_t* messageData, uint32_t messageLength, N_Result nResult, Mtype mtype) { \
    RECEPTION_END_FLAG_NAME = true;                                                                                               \
    ASSERT_EQ(nResult, EXPECTED_N_RESULT);                                                                                                            \
    N_AI expected_nai = EXPECTED_N_AI;\
    ASSERT_MESSAGE_METADATA(expected_nai, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE);                                                                 \
    for(uint32_t i = 0; i < messageLength; i++) { \
    ASSERT_EQ(messageData[i], EXPECTED_MESSAGE[i]); \
    }

/// Macro to create the N_USData_confirm callback. It will assert the metadata of the message and the N_Result, set the reception_end_flag to true and after that you can run your custom code below. Don't forget to close the '}' at the end of your custom code
#define N_USData_indication_reception_cb(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE, EXPECTED_MESSAGE, EXPECTED_N_RESULT) \
    N_USData_indication_reception_cb_internal(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE, EXPECTED_MESSAGE, EXPECTED_N_RESULT, reception_end_flag)
#define N_USData_indication_reception_cb_2(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE, EXPECTED_MESSAGE, EXPECTED_N_RESULT) \
    N_USData_indication_reception_cb_internal(TEST_NAME##_2, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE, EXPECTED_MESSAGE, EXPECTED_N_RESULT, reception_end_flag_2)

/// Macro to create the N_USData_confirm callback. It will assert the metadata of the message and the N_Result, set the reception_end_flag to true and after that you can run your custom code below. Dont forget to close the '}' at the end of your custom code
#define N_USData_FF_indication_reception_cb(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE) \
    N_USData_FF_indication_reception_cb_internal(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE)
#define N_USData_FF_indication_reception_cb_2(TEST_NAME, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE) \
    N_USData_FF_indication_reception_cb_internal(TEST_NAME##_2, EXPECTED_N_AI, EXPECTED_MESSAGE_LENGTH, EXPECTED_M_TYPE)

/// Macro to create the N_USData_confirm callback. It will assert the metadata of the message and the N_Result, set the reception_end_flag to true and after that you can run your custom code below. Dont forget to close the '}' at the end of your custom code
#define N_USData_confirm_cb(TEST_NAME, EXPECTED_N_AI, EXPECTED_N_RESULT, EXPECTED_M_TYPE) N_USData_confirm_cb_internal(TEST_NAME, EXPECTED_N_AI, EXPECTED_N_RESULT, EXPECTED_M_TYPE, send_end_flag)
#define N_USData_confirm_cb_2(TEST_NAME, EXPECTED_N_AI, EXPECTED_N_RESULT, EXPECTED_M_TYPE) \
    N_USData_confirm_cb_internal(TEST_NAME##_2, EXPECTED_N_AI, EXPECTED_N_RESULT, EXPECTED_M_TYPE, send_end_flag)                                                                                     \


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

// START TEST message_SF_transmission_with_physical_N_TA
const uint8_t message_SF_transmission_with_physical_N_TA[] = "123456"; // 7 bytes (max SF_DL)
const uint32_t messageLength_message_SF_transmission_with_physical_N_TA = sizeof(message_SF_transmission_with_physical_N_TA);
typeof(N_AI::N_SA) N_SA_REC_message_SF_transmission_with_physical_N_TA = 210;
typeof(N_AI::N_SA) N_SA_SEND_message_SF_transmission_with_physical_N_TA = 123;
N_AI N_AI_message_SF_transmission_with_physical_N_TA = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Physical, N_SA_REC_message_SF_transmission_with_physical_N_TA, N_SA_SEND_message_SF_transmission_with_physical_N_TA);

N_USData_indication_reception_cb(message_SF_transmission_with_physical_N_TA, N_AI_message_SF_transmission_with_physical_N_TA, messageLength_message_SF_transmission_with_physical_N_TA, Mtype_Diagnostics, message_SF_transmission_with_physical_N_TA, N_Result::N_OK)

}

N_USData_FF_indication_reception_cb(message_SF_transmission_with_physical_N_TA, N_AI_message_SF_transmission_with_physical_N_TA, messageLength_message_SF_transmission_with_physical_N_TA, Mtype_Diagnostics)

}

N_USData_confirm_cb(message_SF_transmission_with_physical_N_TA, N_AI_message_SF_transmission_with_physical_N_TA, N_Result::N_OK, Mtype_Diagnostics)

}

TEST(DoCANCpp, message_SF_transmission_with_physical_N_TA)
{
    INIT_DEFAULT_STRUCTURES(N_SA_SEND_message_SF_transmission_with_physical_N_TA)
    CANShim* canShimReception = network.newCANShimConnection();
    ASSERT_NE(canShimReception, nullptr);

    DoCANCpp doCan(N_SA_SEND_message_SF_transmission_with_physical_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_SF_transmission_with_physical_N_TA, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShimReception);
    DoCANCpp doCanReception(N_SA_REC_message_SF_transmission_with_physical_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_SF_transmission_with_physical_N_TA, N_USData_FF_indication_reception_cb_message_SF_transmission_with_physical_N_TA, *osShim, *canShimReception);

    std::thread receptionThread(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called

    doCan.N_USData_request(N_SA_REC_message_SF_transmission_with_physical_N_TA, N_TAtype::CAN_CLASSIC_29bit_Physical, (uint8_t*)message_SF_transmission_with_physical_N_TA, messageLength_message_SF_transmission_with_physical_N_TA);

    send_loop(doCan, true); // expect the confirm callback to be called

    receptionThread.join();

    ASSERT_TRUE(N_USData_FF_indication_callback_called_message_SF_transmission_with_physical_N_TA); // expect the FF indication callback to be called
}
// END TEST message_SF_transmission_with_physical_N_TA

// START TEST message_MF_transmission_with_physical_N_TA
const uint8_t message_MF_transmission_with_physical_N_TA[] = "123456789_123456789_123456789_123456789_123456789_"; // more than 7 bytes (max SF_DL)
const uint32_t messageLength_message_MF_transmission_with_physical_N_TA = sizeof(message_MF_transmission_with_physical_N_TA);
typeof(N_AI::N_SA) N_SA_REC_message_MF_transmission_with_physical_N_TA = 210;
typeof(N_AI::N_SA) N_SA_SEND_message_MF_transmission_with_physical_N_TA = 123;
N_AI N_AI_message_MF_transmission_with_physical_N_TA = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Physical, N_SA_REC_message_MF_transmission_with_physical_N_TA, N_SA_SEND_message_MF_transmission_with_physical_N_TA);

N_USData_indication_reception_cb(message_MF_transmission_with_physical_N_TA, N_AI_message_MF_transmission_with_physical_N_TA, messageLength_message_MF_transmission_with_physical_N_TA, Mtype_Diagnostics, message_MF_transmission_with_physical_N_TA, N_Result::N_OK)

}

N_USData_FF_indication_reception_cb(message_MF_transmission_with_physical_N_TA, N_AI_message_MF_transmission_with_physical_N_TA, messageLength_message_MF_transmission_with_physical_N_TA, Mtype_Diagnostics)

}

N_USData_confirm_cb(message_MF_transmission_with_physical_N_TA, N_AI_message_MF_transmission_with_physical_N_TA, N_Result::N_OK, Mtype_Diagnostics)

}

TEST(DoCANCpp, message_MF_transmission_with_physical_N_TA)
{
    INIT_DEFAULT_STRUCTURES(N_SA_SEND_message_MF_transmission_with_physical_N_TA)
    CANShim* canShimReception = network.newCANShimConnection();
    ASSERT_NE(canShimReception, nullptr);

    DoCANCpp doCan(N_SA_SEND_message_MF_transmission_with_physical_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_MF_transmission_with_physical_N_TA, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShimReception);
    DoCANCpp doCanReception(N_SA_REC_message_MF_transmission_with_physical_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_MF_transmission_with_physical_N_TA, N_USData_FF_indication_reception_cb_message_MF_transmission_with_physical_N_TA, *osShim, *canShimReception);

    std::thread receptionThread(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called

    doCan.N_USData_request(N_SA_REC_message_MF_transmission_with_physical_N_TA, N_TAtype::CAN_CLASSIC_29bit_Physical, (uint8_t*)message_MF_transmission_with_physical_N_TA, messageLength_message_MF_transmission_with_physical_N_TA);

    send_loop(doCan, true); // expect the confirm callback to be called

    receptionThread.join();

    ASSERT_TRUE(N_USData_FF_indication_callback_called_message_MF_transmission_with_physical_N_TA); // expect the FF indication callback to be called
}
// END TEST message_MF_transmission_with_physical_N_TA

// START TEST message_transmission_with_physical_N_TA_min_message
const uint8_t message_message_transmission_with_physical_N_TA_min_message[] = ""; // 1 byte (less than min SF_DL)
const uint32_t messageLength_message_transmission_with_physical_N_TA_min_message = sizeof(message_message_transmission_with_physical_N_TA_min_message);
typeof(N_AI::N_SA) N_SA_REC_message_transmission_with_physical_N_TA_min_message = 210;
typeof(N_AI::N_SA) N_SA_SEND_message_transmission_with_physical_N_TA_min_message = 123;
N_AI N_AI_message_transmission_with_physical_N_TA_min_message = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Physical, N_SA_REC_message_transmission_with_physical_N_TA_min_message, N_SA_SEND_message_transmission_with_physical_N_TA_min_message);

N_USData_indication_reception_cb(message_transmission_with_physical_N_TA_min_message, N_AI_message_transmission_with_physical_N_TA_min_message, messageLength_message_transmission_with_physical_N_TA_min_message, Mtype_Diagnostics, message_message_transmission_with_physical_N_TA_min_message, N_Result::N_OK)

}

N_USData_FF_indication_reception_cb(message_transmission_with_physical_N_TA_min_message, N_AI_message_transmission_with_physical_N_TA_min_message, messageLength_message_transmission_with_physical_N_TA_min_message, Mtype_Diagnostics)

}

N_USData_confirm_cb(message_transmission_with_physical_N_TA_min_message, N_AI_message_transmission_with_physical_N_TA_min_message, N_Result::N_OK, Mtype_Diagnostics)

}

TEST(DoCANCpp, message_transmission_with_physical_N_TA_min_message)
{
    INIT_DEFAULT_STRUCTURES(N_SA_SEND_message_transmission_with_physical_N_TA_min_message)
    CANShim* canShimReception = network.newCANShimConnection();
    ASSERT_NE(canShimReception, nullptr);

    DoCANCpp doCan(N_SA_SEND_message_transmission_with_physical_N_TA_min_message, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_transmission_with_physical_N_TA_min_message, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShimReception);
    DoCANCpp doCanReception(N_SA_REC_message_transmission_with_physical_N_TA_min_message, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_transmission_with_physical_N_TA_min_message, N_USData_FF_indication_reception_cb_message_transmission_with_physical_N_TA_min_message, *osShim, *canShimReception);

    std::thread receptionThread(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called

    doCan.N_USData_request(N_SA_REC_message_transmission_with_physical_N_TA_min_message, N_TAtype::CAN_CLASSIC_29bit_Physical, (uint8_t*)message_message_transmission_with_physical_N_TA_min_message, messageLength_message_transmission_with_physical_N_TA_min_message);

    send_loop(doCan, true); // expect the confirm callback to be called

    receptionThread.join();

    ASSERT_FALSE(N_USData_FF_indication_callback_called_message_transmission_with_physical_N_TA_min_message); // expect the FF indication callback to not be called
}
// END TEST message_transmission_with_physical_N_TA_min_message

// START TEST message_transmission_with_functional_N_TA

const uint8_t message_message_transmission_with_functional_N_TA[] = "123456"; // 7 bytes (max SF_DL)
const uint32_t messageLength_message_transmission_with_functional_N_TA = sizeof(message_message_transmission_with_functional_N_TA);
typeof(N_AI::N_SA) N_SA_REC_message_transmission_with_functional_N_TA = 210;
typeof(N_AI::N_SA) N_SA_SEND_message_transmission_with_functional_N_TA = 123;
N_AI N_AI_message_transmission_with_functional_N_TA = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Functional, N_SA_REC_message_transmission_with_functional_N_TA, N_SA_SEND_message_transmission_with_functional_N_TA);

N_USData_indication_reception_cb(message_transmission_with_functional_N_TA, N_AI_message_transmission_with_functional_N_TA, messageLength_message_transmission_with_functional_N_TA, Mtype_Diagnostics, message_message_transmission_with_functional_N_TA, N_Result::N_OK)

}

N_USData_confirm_cb(message_transmission_with_functional_N_TA, N_AI_message_transmission_with_functional_N_TA, N_Result::N_OK, Mtype_Diagnostics)

}

N_USData_FF_indication_reception_cb(message_transmission_with_functional_N_TA, N_AI_message_transmission_with_functional_N_TA, messageLength_message_transmission_with_functional_N_TA, Mtype_Diagnostics)

}

TEST(DoCANCpp, message_transmission_with_functional_N_TA)
{
    INIT_DEFAULT_STRUCTURES(N_SA_SEND_message_transmission_with_functional_N_TA)
    CANShim* canShimReception = network.newCANShimConnection();
    ASSERT_NE(canShimReception, nullptr);
    CANShim* canShimReception2 = network.newCANShimConnection();
    ASSERT_NE(canShimReception2, nullptr);

    DoCANCpp doCan(N_SA_SEND_message_transmission_with_functional_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_transmission_with_functional_N_TA, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShimReception);
    DoCANCpp doCanReception(N_SA_REC_message_transmission_with_functional_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_transmission_with_functional_N_TA, N_USData_FF_indication_reception_cb_message_transmission_with_functional_N_TA, *osShim, *canShimReception);

    std::thread receptionThread(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called

    doCan.N_USData_request(N_SA_REC_message_transmission_with_functional_N_TA, N_TAtype::CAN_CLASSIC_29bit_Functional, (uint8_t*)message_message_transmission_with_functional_N_TA, messageLength_message_transmission_with_functional_N_TA);

    send_loop(doCan, true); // expect the confirm callback to be called

    receptionThread.join();

    ASSERT_FALSE(N_USData_FF_indication_callback_called_message_transmission_with_functional_N_TA); // expect the FF indication callback to not be called
}
// END TEST message_transmission_with_functional_N_TA

// START TEST message_transmission_with_functional_N_TA_multicast

const uint8_t message_message_transmission_with_functional_N_TA_multicast[] = "123456"; // 7 bytes (max SF_DL)
const uint32_t messageLength_message_transmission_with_functional_N_TA_multicast = sizeof(message_message_transmission_with_functional_N_TA_multicast);
typeof(N_AI::N_SA) N_SA_REC_message_transmission_with_functional_N_TA_multicast = 210;
typeof(N_AI::N_SA) N_SA_SEND_message_transmission_with_functional_N_TA_multicast = 123;
N_AI N_AI_message_transmission_with_functional_N_TA_multicast = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Functional, N_SA_REC_message_transmission_with_functional_N_TA_multicast, N_SA_SEND_message_transmission_with_functional_N_TA_multicast);

N_USData_indication_reception_cb(message_transmission_with_functional_N_TA_multicast, N_AI_message_transmission_with_functional_N_TA_multicast, messageLength_message_transmission_with_functional_N_TA_multicast, Mtype_Diagnostics, message_message_transmission_with_functional_N_TA_multicast, N_Result::N_OK)

}

N_USData_confirm_cb(message_transmission_with_functional_N_TA_multicast, N_AI_message_transmission_with_functional_N_TA_multicast, N_Result::N_OK, Mtype_Diagnostics)

}

N_USData_FF_indication_reception_cb(message_transmission_with_functional_N_TA_multicast, N_AI_message_transmission_with_functional_N_TA_multicast, messageLength_message_transmission_with_functional_N_TA_multicast, Mtype_Diagnostics)

}

N_USData_indication_reception_cb_2(message_transmission_with_functional_N_TA_multicast, N_AI_message_transmission_with_functional_N_TA_multicast, messageLength_message_transmission_with_functional_N_TA_multicast, Mtype_Diagnostics, message_message_transmission_with_functional_N_TA_multicast, N_Result::N_OK)

}

N_USData_confirm_cb_2(message_transmission_with_functional_N_TA_multicast, N_AI_message_transmission_with_functional_N_TA_multicast, N_Result::N_OK, Mtype_Diagnostics)

}

N_USData_FF_indication_reception_cb_2(message_transmission_with_functional_N_TA_multicast, N_AI_message_transmission_with_functional_N_TA_multicast, messageLength_message_transmission_with_functional_N_TA_multicast, Mtype_Diagnostics)

}

TEST(DoCANCpp, message_transmission_with_functional_N_TA_multicast)
{
    INIT_DEFAULT_STRUCTURES(N_SA_SEND_message_transmission_with_functional_N_TA_multicast)
    CANShim* canShimReception = network.newCANShimConnection();
    ASSERT_NE(canShimReception, nullptr);
    CANShim* canShimReception2 = network.newCANShimConnection();
    ASSERT_NE(canShimReception2, nullptr);

    DoCANCpp doCan(N_SA_SEND_message_transmission_with_functional_N_TA_multicast, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_transmission_with_functional_N_TA_multicast, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShimReception);
    DoCANCpp doCanReception(N_SA_REC_message_transmission_with_functional_N_TA_multicast, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_transmission_with_functional_N_TA_multicast, N_USData_FF_indication_reception_cb_message_transmission_with_functional_N_TA_multicast, *osShim, *canShimReception);
    DoCANCpp doCanReception2(N_SA_REC_message_transmission_with_functional_N_TA_multicast, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_transmission_with_functional_N_TA_multicast_2, N_USData_FF_indication_reception_cb_message_transmission_with_functional_N_TA_multicast_2, *osShim, *canShimReception2);

    std::thread receptionThread(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called
    std::thread receptionThread2(receptionThreadFunction_2, &doCanReception2, true); // expect the indication callback to be called

    doCan.N_USData_request(N_SA_REC_message_transmission_with_functional_N_TA_multicast, N_TAtype::CAN_CLASSIC_29bit_Functional, (uint8_t*)message_message_transmission_with_functional_N_TA_multicast, messageLength_message_transmission_with_functional_N_TA_multicast);

    send_loop(doCan, true); // expect the confirm callback to be called

    receptionThread.join();
    receptionThread2.join();

    ASSERT_FALSE(N_USData_FF_indication_callback_called_message_transmission_with_functional_N_TA_multicast); // expect the FF indication callback to not be called
    ASSERT_FALSE(N_USData_FF_indication_callback_called_message_transmission_with_functional_N_TA_multicast_2); // expect the FF indication callback to not be called
}
// END TEST message_transmission_with_functional_N_TA_multicast

// START TEST message_transmission_with_functional_N_TA_bigger_than_SF_DL

const uint8_t message_message_transmission_with_functional_N_TA_bigger_than_SF_DL[] = "1234567"; // 8 bytes (bigger than SF_DL)
const uint32_t messageLength_message_transmission_with_functional_N_TA_bigger_than_SF_DL = sizeof(message_message_transmission_with_functional_N_TA_bigger_than_SF_DL);
typeof(N_AI::N_SA) N_SA_REC_message_transmission_with_functional_N_TA_bigger_than_SF_DL = 210;
typeof(N_AI::N_SA) N_SA_SEND_message_transmission_with_functional_N_TA_bigger_than_SF_DL = 123;
N_AI N_AI_message_transmission_with_functional_N_TA_bigger_than_SF_DL = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Functional, N_SA_REC_message_transmission_with_functional_N_TA_bigger_than_SF_DL, N_SA_SEND_message_transmission_with_functional_N_TA_bigger_than_SF_DL);

N_USData_indication_reception_cb(message_transmission_with_functional_N_TA_bigger_than_SF_DL, N_AI_message_transmission_with_functional_N_TA_bigger_than_SF_DL, messageLength_message_transmission_with_functional_N_TA_bigger_than_SF_DL, Mtype_Diagnostics, message_message_transmission_with_functional_N_TA_bigger_than_SF_DL, N_Result::N_ERROR)

}

N_USData_confirm_cb(message_transmission_with_functional_N_TA_bigger_than_SF_DL, N_AI_message_transmission_with_functional_N_TA_bigger_than_SF_DL, N_Result::N_ERROR, Mtype_Diagnostics)

}

N_USData_FF_indication_reception_cb(message_transmission_with_functional_N_TA_bigger_than_SF_DL, N_AI_message_transmission_with_functional_N_TA_bigger_than_SF_DL, messageLength_message_transmission_with_functional_N_TA_bigger_than_SF_DL, Mtype_Diagnostics)

}

TEST(DoCANCpp, message_transmission_with_functional_N_TA_bigger_than_SF_DL)
{
    INIT_DEFAULT_STRUCTURES(N_SA_SEND_message_transmission_with_functional_N_TA_bigger_than_SF_DL)
    CANShim* canShimReception = network.newCANShimConnection();
    ASSERT_NE(canShimReception, nullptr);

    DoCANCpp doCan(N_SA_SEND_message_transmission_with_functional_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_transmission_with_functional_N_TA_bigger_than_SF_DL, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShimReception);
    DoCANCpp doCanReception(N_SA_REC_message_transmission_with_functional_N_TA, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_transmission_with_functional_N_TA_bigger_than_SF_DL, N_USData_FF_indication_reception_cb_message_transmission_with_functional_N_TA_bigger_than_SF_DL, *osShim, *canShimReception);

    std::thread receptionThread(receptionThreadFunction, &doCanReception, false); // expect the indication callback not to be called

    doCan.N_USData_request(N_SA_REC_message_transmission_with_functional_N_TA, N_TAtype::CAN_CLASSIC_29bit_Functional, (uint8_t*)message_message_transmission_with_functional_N_TA_bigger_than_SF_DL, messageLength_message_transmission_with_functional_N_TA_bigger_than_SF_DL, Mtype_Diagnostics);
    send_loop(doCan, true); // expect the confirm callback to be called

    receptionThread.join();

    ASSERT_FALSE(N_USData_FF_indication_callback_called_message_transmission_with_functional_N_TA_bigger_than_SF_DL); // expect the FF indication callback not to be called
}
// END TEST message_transmission_with_functional_N_TA_bigger_than_SF_DL

// START TEST message_transmission_with_functional_N_TA_min_message

const uint8_t message_message_transmission_with_functional_N_TA_min_message[] = ""; // 1 byte
const uint32_t messageLength_message_transmission_with_functional_N_TA_min_message = sizeof(message_message_transmission_with_functional_N_TA_min_message);
typeof(N_AI::N_SA) N_SA_REC_message_transmission_with_functional_N_TA_min_message = 210;
typeof(N_AI::N_SA) N_SA_SEND_message_transmission_with_functional_N_TA_min_message = 123;
N_AI N_AI_message_transmission_with_functional_N_TA_min_message = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Functional, N_SA_REC_message_transmission_with_functional_N_TA_min_message, N_SA_SEND_message_transmission_with_functional_N_TA_min_message);

N_USData_confirm_cb(message_transmission_with_functional_N_TA_min_message, N_AI_message_transmission_with_functional_N_TA_min_message, N_Result::N_OK, Mtype_Diagnostics)

}

N_USData_FF_indication_reception_cb(message_transmission_with_functional_N_TA_min_message, N_AI_message_transmission_with_functional_N_TA_min_message, messageLength_message_transmission_with_functional_N_TA_min_message, Mtype_Diagnostics)

}

N_USData_indication_reception_cb(message_transmission_with_functional_N_TA_min_message, N_AI_message_transmission_with_functional_N_TA_min_message, messageLength_message_transmission_with_functional_N_TA_min_message, Mtype_Diagnostics, message_message_transmission_with_functional_N_TA_min_message, N_Result::N_OK)

}

TEST(DoCANCpp, message_transmission_with_functional_N_TA_min_message)
{
    INIT_DEFAULT_STRUCTURES(N_SA_SEND_message_transmission_with_functional_N_TA_min_message)
    CANShim* canShimReception = network.newCANShimConnection();
    ASSERT_NE(canShimReception, nullptr);

    DoCANCpp doCan(N_SA_SEND_message_transmission_with_functional_N_TA_min_message, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_transmission_with_functional_N_TA_min_message, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShimReception);
    DoCANCpp doCanReception(N_SA_REC_message_transmission_with_functional_N_TA_min_message, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_transmission_with_functional_N_TA_min_message, N_USData_FF_indication_reception_cb_message_transmission_with_functional_N_TA_min_message, *osShim, *canShimReception);

    std::thread receptionThread(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called

    doCan.N_USData_request(N_SA_REC_message_transmission_with_functional_N_TA_min_message, N_TAtype::CAN_CLASSIC_29bit_Functional, (uint8_t*)message_message_transmission_with_functional_N_TA_min_message, messageLength_message_transmission_with_functional_N_TA_min_message, Mtype_Diagnostics);
    send_loop(doCan, true); // expect the confirm callback to be called

    receptionThread.join();

    ASSERT_FALSE(N_USData_FF_indication_callback_called_message_transmission_with_functional_N_TA_min_message); // expect the FF indication callback not to be called
}
// END TEST message_transmission_with_functional_N_TA_min_message

// START TEST message_multiple_transmission_with_same_N_AI

const uint8_t message_message_multiple_transmission_with_same_N_AI_1[] = "1234567890"; // 2 frames
const uint8_t message_message_multiple_transmission_with_same_N_AI_2[] = "0987654321"; // 2 frames
const uint32_t messageLength_message_multiple_transmission_with_same_N_AI_1 = sizeof(message_message_multiple_transmission_with_same_N_AI_1);
const uint32_t messageLength_message_multiple_transmission_with_same_N_AI_2 = sizeof(message_message_multiple_transmission_with_same_N_AI_2);
typeof(N_AI::N_SA) N_SA_REC_message_multiple_transmission_with_same_N_AI = 210;
typeof(N_AI::N_SA) N_SA_SEND_message_multiple_transmission_with_same_N_AI = 123;
N_AI N_AI_message_multiple_transmission_with_same_N_AI = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Functional, N_SA_REC_message_multiple_transmission_with_same_N_AI, N_SA_SEND_message_multiple_transmission_with_same_N_AI);

void N_USData_indication_reception_cb_message_multiple_transmission_with_same_N_AI(N_AI nAi, const uint8_t* messageData, uint32_t messageLength, N_Result nResult, Mtype mtype)
{
    static bool first_message = true;

    reception_end_flag = true;
    ASSERT_EQ(nResult, N_Result::N_OK);
    N_AI expected_nai = N_AI_message_multiple_transmission_with_same_N_AI;
    ASSERT_EQ_N_AI(expected_nai, nAi);
    ASSERT_EQ(mtype, Mtype_Diagnostics);
    if(first_message)
    {
        ASSERT_EQ(messageLength, messageLength_message_multiple_transmission_with_same_N_AI_1);
        for(uint32_t i = 0; i < messageLength; i++)
        {
            ASSERT_EQ(messageData[i], message_message_multiple_transmission_with_same_N_AI_1[i]);
        }
        first_message = false;
    }
    else
    {
        ASSERT_EQ(messageLength, messageLength_message_multiple_transmission_with_same_N_AI_2);
        for(uint32_t i = 0; i < messageLength; i++)
        {
            ASSERT_EQ(messageData[i], message_message_multiple_transmission_with_same_N_AI_2[i]);
        }
    }
}

N_USData_confirm_cb(message_multiple_transmission_with_same_N_AI, N_AI_message_multiple_transmission_with_same_N_AI, N_Result::N_OK, Mtype_Diagnostics)

}

volatile bool N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI = 0;
void N_USData_FF_indication_reception_cb_message_multiple_transmission_with_same_N_AI(N_AI nAi, uint32_t messageLength, Mtype mtype)
{
    static bool first_message = true;
    N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI = true;
    N_AI expected_nai = N_AI_message_multiple_transmission_with_same_N_AI;
    ASSERT_EQ_N_AI(expected_nai, nAi);
    ASSERT_EQ(mtype, Mtype_Diagnostics);
    if(first_message)
    {
        ASSERT_EQ(messageLength, messageLength_message_multiple_transmission_with_same_N_AI_1);
        first_message = false;
    }
    else
    {
        ASSERT_EQ(messageLength, messageLength_message_multiple_transmission_with_same_N_AI_2);
    }
}

TEST(DoCANCpp, message_multiple_transmission_with_same_N_AI)
{
    INIT_DEFAULT_STRUCTURES(N_SA_SEND_message_multiple_transmission_with_same_N_AI)
    CANShim* canShimReception = network.newCANShimConnection();
    ASSERT_NE(canShimReception, nullptr);

    DoCANCpp doCan(N_SA_SEND_message_multiple_transmission_with_same_N_AI, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_multiple_transmission_with_same_N_AI, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShim);
    DoCANCpp doCanReception(N_SA_REC_message_multiple_transmission_with_same_N_AI, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_multiple_transmission_with_same_N_AI, N_USData_FF_indication_reception_cb_message_multiple_transmission_with_same_N_AI, *osShim, *canShimReception);

    std::thread receptionThread(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called

    doCan.N_USData_request(N_SA_REC_message_multiple_transmission_with_same_N_AI, N_TAtype::CAN_CLASSIC_29bit_Functional, (uint8_t*)message_message_multiple_transmission_with_same_N_AI_1, messageLength_message_multiple_transmission_with_same_N_AI_1, Mtype_Diagnostics);
    doCan.N_USData_request(N_SA_REC_message_multiple_transmission_with_same_N_AI, N_TAtype::CAN_CLASSIC_29bit_Functional, (uint8_t*)message_message_multiple_transmission_with_same_N_AI_2, messageLength_message_multiple_transmission_with_same_N_AI_2, Mtype_Diagnostics);

    DoCANCpp::run_step(&doCan); // send first frame of the first message
    auxOSShim->osSleep(DoCANCpp_RunPeriod_MS*5); // wait for the reception thread to start receiving the first message

    if(! N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI)
    {
        receptionThread.join(); // Cleanup of the thread before failing the test
    }
    ASSERT_TRUE(N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI); // expect the FF indication callback to be called with the arrival of the first frame of the first message
    N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI = false; // reset the flag

    DoCANCpp::run_step(&doCan); // send the second frame of the first message
    auxOSShim->osSleep(DoCANCpp_RunPeriod_MS*5); // wait for the reception thread to start receiving the second frame of the first message

    if(N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI)
    {
        receptionThread.join(); // Cleanup of the thread before failing the test
    }
    ASSERT_FALSE(N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI); // expect the FF indication callback not to be called

    receptionThread.join(); // wait for the reception thread to finish receiving the first message

    std::thread receptionThread2(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called

    DoCANCpp::run_step(&doCan); // send first frame of the second message
    auxOSShim->osSleep(DoCANCpp_RunPeriod_MS*5); // wait for the reception thread to start receiving the second message

    if(! N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI)
    {
        receptionThread2.join(); // Cleanup of the thread before failing the test
    }
    ASSERT_TRUE(N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI); // expect the FF indication callback to be called with the arrival of the first frame of the first message

    N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI = false; // reset the flag

    DoCANCpp::run_step(&doCan); // send the second frame of the second message
    auxOSShim->osSleep(DoCANCpp_RunPeriod_MS*5); // wait for the reception thread to start receiving the second frame of the second message

    if(N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI)
    {
        receptionThread2.join(); // Cleanup of the thread before failing the test
    }
    ASSERT_FALSE(N_USData_FF_indication_callback_called_message_multiple_transmission_with_same_N_AI); // expect the FF indication callback not to be called

    receptionThread2.join(); // wait for the reception thread to finish receiving the second message
}
// END TEST message_multiple_transmission_with_different_N_AI

// START TEST message_multiple_transmission_with_different_N_AI

const uint8_t message_message_multiple_transmission_with_different_N_AI_1[] = "1234567890"; // 2 frames
const uint8_t message_message_multiple_transmission_with_different_N_AI_2[] = "0987654321"; // 2 frames
const uint32_t messageLength_message_multiple_transmission_with_different_N_AI_1 = sizeof(message_message_multiple_transmission_with_different_N_AI_1);
const uint32_t messageLength_message_multiple_transmission_with_different_N_AI_2 = sizeof(message_message_multiple_transmission_with_different_N_AI_2);
typeof(N_AI::N_SA) N_SA_REC_message_multiple_transmission_with_different_N_AI = 210;
typeof(N_AI::N_SA) N_SA_REC_message_multiple_transmission_with_different_N_AI_2 = 211;
typeof(N_AI::N_SA) N_SA_SEND_message_multiple_transmission_with_different_N_AI = 123;
N_AI N_AI_message_multiple_transmission_with_different_N_AI = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Functional, N_SA_REC_message_multiple_transmission_with_different_N_AI, N_SA_SEND_message_multiple_transmission_with_different_N_AI);
N_AI N_AI_message_multiple_transmission_with_different_N_AI_2 = DoCANCpp_N_AI_CONFIG(N_TAtype::CAN_CLASSIC_29bit_Functional, N_SA_REC_message_multiple_transmission_with_different_N_AI_2, N_SA_SEND_message_multiple_transmission_with_different_N_AI);

void N_USData_indication_reception_cb_message_multiple_transmission_with_different_N_AI(N_AI nAi, const uint8_t* messageData, uint32_t messageLength, N_Result nResult, Mtype mtype)
{
  static bool first_message = true;

  reception_end_flag = true;
  ASSERT_EQ(nResult, N_Result::N_OK);
  ASSERT_EQ(mtype, Mtype_Diagnostics);
  if(first_message)
  {
    N_AI expected_nai = N_AI_message_multiple_transmission_with_different_N_AI;
    ASSERT_EQ_N_AI(nAi, expected_nai);
    ASSERT_EQ(messageLength, messageLength_message_multiple_transmission_with_different_N_AI_1);
    for(uint32_t i = 0; i < messageLength; i++)
    {
      ASSERT_EQ(messageData[i], message_message_multiple_transmission_with_different_N_AI_1[i]);
    }
    first_message = false;
  }
  else
  {
    N_AI expected_nai = N_AI_message_multiple_transmission_with_different_N_AI_2;
    ASSERT_EQ_N_AI(nAi, expected_nai);
    ASSERT_EQ(messageLength, messageLength_message_multiple_transmission_with_different_N_AI_2);
    for(uint32_t i = 0; i < messageLength; i++)
    {
      ASSERT_EQ(messageData[i], message_message_multiple_transmission_with_different_N_AI_2[i]);
    }
  }
}

N_USData_confirm_cb(message_multiple_transmission_with_different_N_AI, N_AI_message_multiple_transmission_with_different_N_AI, N_Result::N_OK, Mtype_Diagnostics)

}

volatile uint8_t N_USData_FF_indication_callback_called_message_multiple_transmission_with_different_N_AI = 0;
void N_USData_FF_indication_reception_cb_message_multiple_transmission_with_different_N_AI(N_AI nAi, uint32_t messageLength, Mtype mtype)
{
  static bool first_message = true;
  N_USData_FF_indication_callback_called_message_multiple_transmission_with_different_N_AI+=1;
  ASSERT_EQ(mtype, Mtype_Diagnostics);
  if(first_message)
  {
    N_AI expected_nai = N_AI_message_multiple_transmission_with_different_N_AI;
    ASSERT_EQ_N_AI(nAi, expected_nai);
    ASSERT_EQ(messageLength, messageLength_message_multiple_transmission_with_different_N_AI_1);
    first_message = false;
  }
  else
  {
    N_AI expected_nai = N_AI_message_multiple_transmission_with_different_N_AI_2;
    ASSERT_EQ_N_AI(nAi, expected_nai);
    ASSERT_EQ(messageLength, messageLength_message_multiple_transmission_with_different_N_AI_2);
  }
}

TEST(DoCANCpp, message_multiple_transmission_with_different_N_AI)
{
  INIT_DEFAULT_STRUCTURES(N_SA_SEND_message_multiple_transmission_with_different_N_AI)
  CANShim* canShimReception = network.newCANShimConnection();
  ASSERT_NE(canShimReception, nullptr);

  DoCANCpp doCan(N_SA_SEND_message_multiple_transmission_with_different_N_AI, totalAvailableMemoryForRunners, N_USData_confirm_cb_message_multiple_transmission_with_different_N_AI, N_USData_indication_dummy_cb, N_USData_FF_indication_dummy_cb, *osShim, *canShim);
  DoCANCpp doCanReception(N_SA_REC_message_multiple_transmission_with_different_N_AI, totalAvailableMemoryForRunners, N_USData_confirm_dummy_cb, N_USData_indication_reception_cb_message_multiple_transmission_with_different_N_AI, N_USData_FF_indication_reception_cb_message_multiple_transmission_with_different_N_AI, *osShim, *canShimReception);

  std::thread receptionThread(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called

  doCan.N_USData_request(N_SA_REC_message_multiple_transmission_with_different_N_AI, N_TAtype::CAN_CLASSIC_29bit_Functional, (uint8_t*)message_message_multiple_transmission_with_different_N_AI_1, messageLength_message_multiple_transmission_with_different_N_AI_1, Mtype_Diagnostics);
  doCan.N_USData_request(N_SA_REC_message_multiple_transmission_with_different_N_AI_2, N_TAtype::CAN_CLASSIC_29bit_Functional, (uint8_t*)message_message_multiple_transmission_with_different_N_AI_2, messageLength_message_multiple_transmission_with_different_N_AI_2, Mtype_Diagnostics);

  DoCANCpp::run_step(&doCan); // send first frame of the first message
  auxOSShim->osSleep(DoCANCpp_RunPeriod_MS*5); // wait for the reception thread to start receiving the first and second message.

  if(! N_USData_FF_indication_callback_called_message_multiple_transmission_with_different_N_AI)
  {
    receptionThread.join(); // Cleanup of the thread before failing the test
  }
  ASSERT_EQ(N_USData_FF_indication_callback_called_message_multiple_transmission_with_different_N_AI, 2); // expect the FF indication callback to be called with the arrival of the first frame of the first message

  DoCANCpp::run_step(&doCan); // send the second frame of the first message
  auxOSShim->osSleep(DoCANCpp_RunPeriod_MS*5); // wait for the reception thread to start receiving the second frame of the first message

  if(N_USData_FF_indication_callback_called_message_multiple_transmission_with_different_N_AI != 2)
  {
    receptionThread.join(); // Cleanup of the thread before failing the test
  }
  ASSERT_EQ(N_USData_FF_indication_callback_called_message_multiple_transmission_with_different_N_AI, 2); // expect the FF indication callback not to be called

  receptionThread.join(); // wait for the reception thread to finish receiving the first message

  std::thread receptionThread2(receptionThreadFunction, &doCanReception, true); // expect the indication callback to be called to finish receiving the second message

  receptionThread2.join(); // wait for the reception thread to finish receiving the second message
}
// END TEST message_multiple_transmission_with_different_N_AI

/*
 * TODO test cases:
 * - multiple messages being transmitted & received at the same time (with the same and different N_TAtype)
 * - test that if the CAN bus is not active, messages running are not transmitted/received and immediately return N_ERROR
 */
