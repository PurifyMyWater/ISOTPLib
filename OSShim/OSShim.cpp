#include "OSShim.h"
#include <cassert>

OSShim::OSShim(OSShim::SleepCallback sleepCallback, OSShim::MillisCallback millisCallback, OSShim::CreateMutexCallback createMutexCallback, OSShim::MallocCallback mallocCallback, OSShim::FreeCallback freeCallback)
{
    if (sleepCallback == nullptr)
    {
        assert(false && "OSShim(): sleepCallback is null");
    }
    if (millisCallback == nullptr)
    {
        assert(false && "OSShim(): millisCallback is null");
    }
    if(createMutexCallback == nullptr)
    {
        assert(false && "OSShim(): createMutexCallback is null");
    }
    if(mallocCallback == nullptr)
    {
        assert(false && "OSShim(): mallocCallback is null");
    }
    if(freeCallback == nullptr)
    {
        assert(false && "OSShim(): freeCallback is null");
    }

    this->sleepCallback = sleepCallback;
    this->millisCallback = millisCallback;
    this->createMutexCallback = createMutexCallback;
    this->mallocCallback = mallocCallback;
    this->freeCallback = freeCallback;
}

void OSShim::sleep(uint32_t ms)
{
    this->sleepCallback(ms);
}

uint32_t OSShim::millis()
{
    return this->millisCallback();
}

OSShim_Mutex* OSShim::createMutex()
{
    return this->createMutexCallback();
}

void* OSShim::malloc(uint32_t size)
{
    return this->mallocCallback(size);
}

void OSShim::free(void* ptr)
{
    this->freeCallback(ptr);
}
