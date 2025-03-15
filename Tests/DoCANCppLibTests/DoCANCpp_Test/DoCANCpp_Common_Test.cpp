#include "DoCANCpp_Common.h"
#include <gtest/gtest.h>

TEST(DoCANCpp_Common, N_Result_to_string)
{
    EXPECT_STREQ("NOT_STARTED", N_Result_to_string(NOT_STARTED));
    EXPECT_STREQ("IN_PROGRESS", N_Result_to_string(IN_PROGRESS));
    EXPECT_STREQ("N_OK", N_Result_to_string(N_OK));
    EXPECT_STREQ("N_TIMEOUT_A", N_Result_to_string(N_TIMEOUT_A));
    EXPECT_STREQ("N_TIMEOUT_Bs", N_Result_to_string(N_TIMEOUT_Bs));
    EXPECT_STREQ("N_TIMEOUT_Cr", N_Result_to_string(N_TIMEOUT_Cr));
    EXPECT_STREQ("N_WRONG_SN", N_Result_to_string(N_WRONG_SN));
    EXPECT_STREQ("N_INVALID_FS", N_Result_to_string(N_INVALID_FS));
    EXPECT_STREQ("N_UNEXP_PDU", N_Result_to_string(N_UNEXP_PDU));
    EXPECT_STREQ("N_WFT_OVRN", N_Result_to_string(N_WFT_OVRN));
    EXPECT_STREQ("N_BUFFER_OVFLW", N_Result_to_string(N_BUFFER_OVFLW));
    EXPECT_STREQ("N_ERROR", N_Result_to_string(N_ERROR));
    EXPECT_STREQ("UNKNOWN", N_Result_to_string(static_cast<N_Result>(999)));
}

TEST(DoCANCpp_Common, getStMinInMs)
{
    STmin stMin1{.value = 10, .unit = usX100};
    EXPECT_EQ(1, getStMinInMs(stMin1));

    STmin stMin2{.value = 10, .unit = ms};
    EXPECT_EQ(10, getStMinInMs(stMin2));

    STmin stMin3{.value = 0, .unit = usX100};
    EXPECT_EQ(0, getStMinInMs(stMin3));
}
