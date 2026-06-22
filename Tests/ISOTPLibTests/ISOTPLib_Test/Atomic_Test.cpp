#include "Atomic.h"
#include <gtest/gtest.h>
#include "LinuxOSInterface.h"

static LinuxOSInterface linuxOSInterface;

 TEST(Atomic, get_success)
 {
     int    expected = 3;
     int    real;
     Atomic<int> value(expected, linuxOSInterface);
     ASSERT_TRUE(value.get(&real));
     ASSERT_EQ(expected, real);
 }

 TEST(Atomic, get_timeout)
 {
     OSInterface_Mutex* mutex = linuxOSInterface.osCreateMutex();
     int    expected = 3;
     int    real = 0;
     Atomic<int> value(expected, mutex);
     ASSERT_TRUE(mutex->wait(10));
     ASSERT_FALSE(value.get(&real));
     ASSERT_EQ(0, real);
     delete mutex;
 }

 TEST(Atomic, get_nullptrMutex)
 {
     int    expected = 3;
     int    real = 0;
     Atomic<int> value(expected, nullptr);
     ASSERT_FALSE(value.get(&real));
     ASSERT_EQ(0, real);
 }

 TEST(Atomic, set_success)
 {
     int    expected = 3;
     int    real;
     Atomic<int> value(0, linuxOSInterface);
     ASSERT_TRUE(value.set(expected));

     ASSERT_TRUE(value.get(&real));
     ASSERT_EQ(expected, real);
 }

TEST(Atomic, set_nullptrMutex)
{
    Atomic<int> value(0, nullptr);
    ASSERT_FALSE(value.set(4));
}

TEST(Atomic, set_timeout)
{
    OSInterface_Mutex* mutex = linuxOSInterface.osCreateMutex();
    int    expected = 3;
    int    real;
    Atomic<int> value(expected, mutex);
    ASSERT_TRUE(mutex->wait(10));
    ASSERT_FALSE(value.set(0));
    mutex->signal();
    ASSERT_TRUE(value.get(&real));
    ASSERT_EQ(expected, real);
    delete mutex;
}
