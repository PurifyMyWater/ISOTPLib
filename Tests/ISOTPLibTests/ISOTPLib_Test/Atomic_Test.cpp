#include "Atomic.h"
#include <gtest/gtest.h>
#include "LinuxOSInterface.h"

static LinuxOSInterface linuxOSInterface;

// TEST(Atomic, get_success)
// {
//     OSInterface_Mutex* mutex = linuxOSInterface.osCreateMutex();
//     int    expected = 3;
//     int    real;
//     Atomic value(expected, nullptr);
//     ASSERT_TRUE(value.get(&real));
//     ASSERT_EQ(expected, real);
// }

// TEST(Atomic, get_timeout)
// {
//
//     int    expected = 3;
//     int    real;
//     Atomic value(expected, linuxOSInterface);
//     ASSERT_TRUE(value.get(&real));
//     ASSERT_EQ(expected, real);
// }
