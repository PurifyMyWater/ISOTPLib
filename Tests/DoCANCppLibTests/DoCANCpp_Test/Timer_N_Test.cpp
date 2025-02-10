#include <gtest/gtest.h>
#include "Timer_N.h"
#include "LinuxOSShim.h"

static LinuxOSShim linuxOSShim;

TEST(Timer_N, startTimer)
{
    Timer_N timer(linuxOSShim);
    timer.startTimer();
    linuxOSShim.osSleep(10);
    ASSERT_GE(15, timer.getElapsedTime_ms());
    ASSERT_LE(9, timer.getElapsedTime_ms());
    linuxOSShim.osSleep(10);
    ASSERT_GE(25, timer.getElapsedTime_ms());
    ASSERT_LE(19, timer.getElapsedTime_ms());
}

TEST(Timer_N, stopTimer)
{
    Timer_N timer(linuxOSShim);
    timer.startTimer();
    linuxOSShim.osSleep(10);
    timer.stopTimer();
    ASSERT_GE(15, timer.getElapsedTime_ms());
    ASSERT_LE(9, timer.getElapsedTime_ms());
    linuxOSShim.osSleep(10);
    ASSERT_GE(15, timer.getElapsedTime_ms());
    ASSERT_LE(9, timer.getElapsedTime_ms());
}

TEST(Timer_N, getStartTimeStamp)
{
    Timer_N timer(linuxOSShim);
    auto stamp = timer.getStartTimeStamp();
    ASSERT_EQ(0, stamp);
    timer.startTimer();
    stamp = timer.getStartTimeStamp();
    linuxOSShim.osSleep(10);
    auto diff = timer.getStartTimeStamp() - stamp;
    ASSERT_EQ(0, diff);
    timer.stopTimer();
    diff = timer.getStartTimeStamp() - stamp;
    ASSERT_EQ(0, diff);
    timer.startTimer();
    diff = timer.getStartTimeStamp() - stamp;
    ASSERT_GE(15, diff);
    ASSERT_LE(9, diff);
}
