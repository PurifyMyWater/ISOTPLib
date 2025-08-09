#include "ISOTP_Common.h"
#include <gtest/gtest.h>

TEST(ISOTP_Common, N_ResultToString)
{
    EXPECT_STREQ("NOT_STARTED", N_ResultToString(NOT_STARTED));
    EXPECT_STREQ("IN_PROGRESS", N_ResultToString(IN_PROGRESS));
    EXPECT_STREQ("N_OK", N_ResultToString(N_OK));
    EXPECT_STREQ("N_TIMEOUT_A", N_ResultToString(N_TIMEOUT_A));
    EXPECT_STREQ("N_TIMEOUT_Bs", N_ResultToString(N_TIMEOUT_Bs));
    EXPECT_STREQ("N_TIMEOUT_Cr", N_ResultToString(N_TIMEOUT_Cr));
    EXPECT_STREQ("N_WRONG_SN", N_ResultToString(N_WRONG_SN));
    EXPECT_STREQ("N_INVALID_FS", N_ResultToString(N_INVALID_FS));
    EXPECT_STREQ("N_UNEXP_PDU", N_ResultToString(N_UNEXP_PDU));
    EXPECT_STREQ("N_WFT_OVRN", N_ResultToString(N_WFT_OVRN));
    EXPECT_STREQ("N_BUFFER_OVFLW", N_ResultToString(N_BUFFER_OVFLW));
    EXPECT_STREQ("N_ERROR", N_ResultToString(N_ERROR));
    EXPECT_STREQ("UNKNOWN", N_ResultToString(static_cast<N_Result>(999)));
}

TEST(ISOTP_Common, getStMinInMs)
{
    STmin stMin1{.value = 10, .unit = usX100};
    EXPECT_EQ(1, getStMinInMs(stMin1));

    STmin stMin2{.value = 10, .unit = ms};
    EXPECT_EQ(10, getStMinInMs(stMin2));

    STmin stMin3{.value = 0, .unit = usX100};
    EXPECT_EQ(0, getStMinInMs(stMin3));
}
