#ifndef DOCANTESTPROJECT_ASSERT_MACROS_H
#define DOCANTESTPROJECT_ASSERT_MACROS_H

#define ASSERT_EQ_N_AI(EXPECTED_N_AI, REAL_N_AI)                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        ASSERT_EQ((REAL_N_AI).N_NFA_Header, (EXPECTED_N_AI).N_NFA_Header);                                             \
        ASSERT_EQ((REAL_N_AI).N_NFA_Padding, (EXPECTED_N_AI).N_NFA_Padding);                                           \
        ASSERT_EQ((REAL_N_AI).N_TAtype, (EXPECTED_N_AI).N_TAtype);                                                     \
        ASSERT_EQ((REAL_N_AI).N_TA, (EXPECTED_N_AI).N_TA);                                                             \
        ASSERT_EQ((REAL_N_AI).N_SA, (EXPECTED_N_AI).N_SA);                                                             \
        ASSERT_EQ((REAL_N_AI).N_AI, (EXPECTED_N_AI).N_AI);                                                             \
    }                                                                                                                  \
    while (0)

#define ASSERT_EQ_ARRAY(EXPECTED_ARRAY, REAL_ARRAY, ARRAY_SIZE)                                                        \
    do                                                                                                                 \
    {                                                                                                                  \
        for (uint32_t i = 0; i < (ARRAY_SIZE); i++)                                                                    \
        {                                                                                                              \
            OSInterfaceLogVerbose("ASSERT_EQ_ARRAY", "REAL_ARRAY[%u]: %c (%02X)\tEXPECTED_ARRAY[%u]: %c (%02X)",       \
                                  (REAL_ARRAY)[i], (REAL_ARRAY)[i], i, (EXPECTED_ARRAY)[i], (EXPECTED_ARRAY)[i], i);   \
            ASSERT_EQ((REAL_ARRAY)[i], (EXPECTED_ARRAY)[i]);                                                           \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

#define ASSERT_EQ_FRAMES(EXPECTED_FRAME, REAL_FRAME)                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        ASSERT_EQ_N_AI((EXPECTED_FRAME).identifier, (REAL_FRAME).identifier);                                          \
        ASSERT_EQ((EXPECTED_FRAME).flags, (REAL_FRAME).flags);                                                         \
        ASSERT_EQ((EXPECTED_FRAME).data_length_code, (REAL_FRAME).data_length_code);                                   \
        ASSERT_EQ_ARRAY((EXPECTED_FRAME).data, (REAL_FRAME).data, (EXPECTED_FRAME).data_length_code);                  \
    }                                                                                                                  \
    while (0)

#define EXPECT_EQ_N_AI(EXPECTED_N_AI, REAL_N_AI)                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        EXPECT_EQ((REAL_N_AI).N_NFA_Header, (EXPECTED_N_AI).N_NFA_Header);                                             \
        EXPECT_EQ((REAL_N_AI).N_NFA_Padding, (EXPECTED_N_AI).N_NFA_Padding);                                           \
        EXPECT_EQ((REAL_N_AI).N_TAtype, (EXPECTED_N_AI).N_TAtype);                                                     \
        EXPECT_EQ((REAL_N_AI).N_TA, (EXPECTED_N_AI).N_TA);                                                             \
        EXPECT_EQ((REAL_N_AI).N_SA, (EXPECTED_N_AI).N_SA);                                                             \
        EXPECT_EQ((REAL_N_AI).N_AI, (EXPECTED_N_AI).N_AI);                                                             \
    }                                                                                                                  \
    while (0)

#define EXPECT_EQ_ARRAY(EXPECTED_ARRAY, REAL_ARRAY, ARRAY_SIZE)                                                        \
    do                                                                                                                 \
    {                                                                                                                  \
        for (uint32_t i = 0; i < (ARRAY_SIZE); i++)                                                                    \
        {                                                                                                              \
            EXPECT_EQ((REAL_ARRAY)[i], (EXPECTED_ARRAY)[i]);                                                           \
        }                                                                                                              \
    }                                                                                                                  \
    while (0)

#define EXPECT_EQ_FRAMES(EXPECTED_FRAME, REAL_FRAME)                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        EXPECT_EQ_N_AI((EXPECTED_FRAME).identifier, (REAL_FRAME).identifier);                                          \
        EXPECT_EQ((EXPECTED_FRAME).flags, (REAL_FRAME).flags);                                                         \
        EXPECT_EQ((EXPECTED_FRAME).data_length_code, (REAL_FRAME).data_length_code);                                   \
        EXPECT_EQ_ARRAY((EXPECTED_FRAME).data, (REAL_FRAME).data, (EXPECTED_FRAME).data_length_code);                  \
    }                                                                                                                  \
    while (0)

#endif // DOCANTESTPROJECT_ASSERT_MACROS_H
