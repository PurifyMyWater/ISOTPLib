#ifndef DOCANTESTPROJECT_ASSERT_MACROS_H
#define DOCANTESTPROJECT_ASSERT_MACROS_H

#define ASSERT_EQ_N_AI(REAL_N_AI, EXPECTED_N_AI) do{ \
    ASSERT_EQ((REAL_N_AI).N_NFA_Header, (EXPECTED_N_AI).N_NFA_Header); \
    ASSERT_EQ((REAL_N_AI).N_NFA_Padding, (EXPECTED_N_AI).N_NFA_Padding); \
    ASSERT_EQ((REAL_N_AI).N_TAtype, (EXPECTED_N_AI).N_TAtype); \
    ASSERT_EQ((REAL_N_AI).N_TA, (EXPECTED_N_AI).N_TA); \
    ASSERT_EQ((REAL_N_AI).N_SA, (EXPECTED_N_AI).N_SA); \
    ASSERT_EQ((REAL_N_AI).N_AI, (EXPECTED_N_AI).N_AI); } while(0)

#define ASSERT_EQ_FRAMES(REAL_FRAME, EXPECTED_FRAME) \
    ASSERT_EQ_N_AI((REAL_FRAME).identifier, (EXPECTED_FRAME).identifier); \
    ASSERT_EQ((REAL_FRAME).flags, (EXPECTED_FRAME).flags); \
    ASSERT_EQ((REAL_FRAME).data_length_code, (EXPECTED_FRAME).data_length_code); \
    for (uint32_t i = 0; i < (REAL_FRAME).data_length_code && i < (EXPECTED_FRAME).data_length_code; i++) \
    { \
        ASSERT_EQ((REAL_FRAME).data[i], (EXPECTED_FRAME).data[i]); \
    }

#endif //DOCANTESTPROJECT_ASSERT_MACROS_H
