#include "gtest/gtest.h"
#include "LinuxOSShim.h"

TEST(LinuxOSShim, timeTest)
{
    uint32_t timeToSleep = 50;
    uint32_t repeat = 50;
    bool flag = false;

    for (uint32_t i = 0; i < repeat; i++)
    {
        uint32_t millis = linuxOSShim.millis();
        linuxOSShim.sleep(timeToSleep);
        uint32_t millis2 = linuxOSShim.millis();

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
    OSShim_Mutex* mutex = linuxOSShim.createMutex();
    EXPECT_TRUE(mutex != nullptr);
    EXPECT_TRUE(mutex->wait(1000));
}

TEST(LinuxOSShim, mutexSignal)
{
    OSShim_Mutex* mutex = linuxOSShim.createMutex();
    EXPECT_TRUE(mutex != nullptr);
    mutex->signal();
}

TEST(LinuxOSShim, mutexTestNormal)
{
    OSShim_Mutex* mutex = linuxOSShim.createMutex();
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
    OSShim_Mutex* mutex = linuxOSShim.createMutex();
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

TEST(LinuxOSShim, mallocSimpleAlloc)
{
    void* ptr = linuxOSShim.malloc(100);
    ASSERT_NE(ptr, nullptr);
    linuxOSShim.free(ptr);
}

TEST(LinuxOSShim, mallocZeroAlloc)
{
    void* ptr = linuxOSShim.malloc(0);
    ASSERT_NE(ptr, nullptr);
    linuxOSShim.free(ptr);
}

TEST(LinuxOSShim, mallocLargeAlloc)
{
    void* ptr = linuxOSShim.malloc(1024*1024*6);
    ASSERT_NE(ptr, nullptr);
    linuxOSShim.free(ptr);
}

TEST(LinuxOSShim, mallocMultipleAlloc)
{
    void* ptr1 = linuxOSShim.malloc(100);
    void* ptr2 = linuxOSShim.malloc(100);
    void* ptr3 = linuxOSShim.malloc(100);
    ASSERT_NE(ptr1, nullptr);
    ASSERT_NE(ptr2, nullptr);
    ASSERT_NE(ptr3, nullptr);
    linuxOSShim.free(ptr1);
    linuxOSShim.free(ptr2);
    linuxOSShim.free(ptr3);
}
