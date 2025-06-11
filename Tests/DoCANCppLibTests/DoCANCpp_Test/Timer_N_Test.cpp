#include "Timer_N.h"
#include <gtest/gtest.h>
#include "LinuxOSInterface.h"

static LinuxOSInterface linuxOSInterface;

TEST(Timer_N, startTimer)
{
    Timer_N timer(linuxOSInterface);
    timer.startTimer();
    linuxOSInterface.osSleep(10);
    ASSERT_GE(15, timer.getElapsedTime_ms());
    ASSERT_LE(9, timer.getElapsedTime_ms());
    linuxOSInterface.osSleep(10);
    ASSERT_GE(25, timer.getElapsedTime_ms());
    ASSERT_LE(19, timer.getElapsedTime_ms());
}

TEST(Timer_N, stopTimer)
{
    Timer_N timer(linuxOSInterface);
    ASSERT_FALSE(timer.isTimerRunning());
    timer.startTimer();
    ASSERT_TRUE(timer.isTimerRunning());
    linuxOSInterface.osSleep(10);
    timer.stopTimer();
    ASSERT_FALSE(timer.isTimerRunning());
    ASSERT_GE(15, timer.getElapsedTime_ms());
    ASSERT_LE(9, timer.getElapsedTime_ms());
    linuxOSInterface.osSleep(10);
    ASSERT_GE(15, timer.getElapsedTime_ms());
    ASSERT_LE(9, timer.getElapsedTime_ms());
}

TEST(Timer_N, clearTimer)
{
    Timer_N timer(linuxOSInterface);
    ASSERT_FALSE(timer.isTimerRunning());
    timer.startTimer();
    ASSERT_TRUE(timer.isTimerRunning());
    linuxOSInterface.osSleep(10);
    timer.stopTimer();
    ASSERT_FALSE(timer.isTimerRunning());
    ASSERT_GE(15, timer.getElapsedTime_ms());
    ASSERT_LE(9, timer.getElapsedTime_ms());
    timer.clearTimer();
    ASSERT_EQ(0, timer.getElapsedTime_ms());
}

TEST(Timer_N, getStartTimeStamp)
{
    Timer_N timer(linuxOSInterface);
    auto    stamp = timer.getStartTimeStamp();
    ASSERT_EQ(0, stamp);
    timer.startTimer();
    stamp = timer.getStartTimeStamp();
    linuxOSInterface.osSleep(10);
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
