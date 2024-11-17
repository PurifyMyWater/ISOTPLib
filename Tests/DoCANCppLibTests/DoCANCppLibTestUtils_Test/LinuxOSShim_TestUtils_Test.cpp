#include "gtest/gtest.h"
#include "LinuxOSShim.h"

static LinuxOSShim linuxOSShim;

TEST(LinuxOSShim, timeTest)
{
    uint32_t timeToSleep = 50;
    uint32_t repeat = 50;
    bool flag = false;

    for (uint32_t i = 0; i < repeat; i++)
    {
        uint32_t millis = linuxOSShim.osMillis();
        linuxOSShim.osSleep(timeToSleep);
        uint32_t millis2 = linuxOSShim.osMillis();

        EXPECT_GE(millis2, millis + timeToSleep);
        if (millis2 >= millis + timeToSleep + 100)
        {
            printf("Warning: Sleep slept for too long (%dms more than expected)\n", millis2 - millis - timeToSleep);
            flag = true;
        }
    }

    if (flag)
    {
        FAIL() << "Sleep slept for too long";
    }
}

TEST(LinuxOSShim, mutexWait)
{
    OSShim_Mutex* mutex = linuxOSShim.osCreateMutex();
    EXPECT_TRUE(mutex != nullptr);
    EXPECT_TRUE(mutex->wait(1000));
}

TEST(LinuxOSShim, mutexSignal)
{
    OSShim_Mutex* mutex = linuxOSShim.osCreateMutex();
    EXPECT_TRUE(mutex != nullptr);
    mutex->signal();
}

TEST(LinuxOSShim, mutexTestNormal)
{
    OSShim_Mutex* mutex = linuxOSShim.osCreateMutex();
    ASSERT_NE(mutex, nullptr);

    // Lock the mutex
    ASSERT_TRUE(mutex->wait(100000));
    // Flag to check if the second thread was able to lock the mutex
    volatile bool secondThreadLocked = false;

    // Create a second thread that tries to lock the mutex
    std::thread t([&]()
    {
        EXPECT_TRUE(mutex->wait(100000));
        secondThreadLocked = true;
        mutex->signal();

    });

    // Sleep for a short time to ensure the second thread attempts to lock the mutex
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // The second thread should not have been able to lock the mutex yet
    EXPECT_FALSE(secondThreadLocked);

    // Unlock the mutex
    mutex->signal();

    // Wait for the second thread to finish
    t.join();

    // Now the second thread should have been able to lock the mutex
    EXPECT_TRUE(secondThreadLocked);
}

TEST(LinuxOSShim, mutexTestTimeout)
{
    OSShim_Mutex* mutex = linuxOSShim.osCreateMutex();
    ASSERT_NE(mutex, nullptr);

    // Lock the mutex
    ASSERT_TRUE(mutex->wait(100000));
    // Flag to check if the second thread was able to lock the mutex
    volatile bool secondThreadLocked = false;

    // Create a second thread that tries to lock the mutex
    std::thread t([&]()
                  {
                      auto res = mutex->wait(1000);
                      EXPECT_FALSE(res);
                      if(res)
                      {
                        secondThreadLocked = true;
                        mutex->signal();
                      }

                  });

    // Sleep for a short time to ensure the second thread attempts to lock the mutex
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // The second thread should not have been able to lock the mutex yet
    EXPECT_FALSE(secondThreadLocked);

    // Unlock the mutex
    mutex->signal();

    // Wait for the second thread to finish
    t.join();

    EXPECT_FALSE(secondThreadLocked);
}

TEST(LinuxOSShim, osMallocSimpleAlloc)
{
    void* ptr = linuxOSShim.osMalloc(100);
    ASSERT_NE(ptr, nullptr);
    linuxOSShim.osFree(ptr);
}

TEST(LinuxOSShim, osMallocZeroAlloc)
{
    void* ptr = linuxOSShim.osMalloc(0);
    ASSERT_NE(ptr, nullptr);
    linuxOSShim.osFree(ptr);
}

TEST(LinuxOSShim, osMallocLargeAlloc)
{
    void* ptr = linuxOSShim.osMalloc(1024*1024*6);
    ASSERT_NE(ptr, nullptr);
    linuxOSShim.osFree(ptr);
}

TEST(LinuxOSShim, osMallocMultipleAlloc)
{
    void* ptr1 = linuxOSShim.osMalloc(100);
    void* ptr2 = linuxOSShim.osMalloc(100);
    void* ptr3 = linuxOSShim.osMalloc(100);
    ASSERT_NE(ptr1, nullptr);
    ASSERT_NE(ptr2, nullptr);
    ASSERT_NE(ptr3, nullptr);
    linuxOSShim.osFree(ptr1);
    linuxOSShim.osFree(ptr2);
    linuxOSShim.osFree(ptr3);
}
