#include <stdlib.h>
#include <mutex>
#include <time.h>
#include "OSShim.h"
#include "OSShim_Mutex.h"

class linuxMutex : public OSShim_Mutex
{
public:
    linuxMutex()
    {
        pthread_mutex_init(&mutex, nullptr);
    }
    ~linuxMutex()
    {
        pthread_mutex_destroy(&mutex);
    }
    void signal() override
    {
        pthread_mutex_unlock(&mutex);
    }
    bool wait(uint32_t max_time_to_wait_ms) override
    {
        struct timespec ts{};
        ts.tv_nsec = max_time_to_wait_ms * 1000000;

        return pthread_mutex_timedlock(&mutex, &ts) == 0;
    }

private:
    pthread_mutex_t mutex;
};

void linuxSleep(uint32_t ms)
{
    timespec ts{};
    ts.tv_nsec = ms * 1000000;
    nanosleep(&ts, nullptr);
}

uint32_t linuxMillis()
{
    timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

OSShim_Mutex* linuxCreateMutex()
{
    return new linuxMutex();
}

void* linuxMalloc(uint32_t size)
{
    return malloc(size);
}

void linuxFree(void* ptr)
{
    free(ptr);
}

OSShim linuxOSShim(linuxSleep, linuxMillis, linuxCreateMutex, linuxMalloc, linuxFree);
