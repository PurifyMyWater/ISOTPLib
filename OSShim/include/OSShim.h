#ifndef OSShim_h
#define OSShim_h

#include <cstdint>
#include "OSShim_Mutex.h"

#ifdef ESP_PLATFORM

    #include "esp_log.h"

    #ifndef OSShimVerbose
    #define OSShimVerbose(tag, format, ...) ESP_LOGV(tag, format, ##__VA_ARGS__)
    #endif

    #ifndef OSShimDebug
    #define OSShimDebug(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
    #endif

    #ifndef OSShimInfo
    #define OSShimInfo(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
    #endif

    #ifndef OSShimWarn
    #define OSShimWarn(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
    #endif

    #ifndef OSShimError
    #define OSShimError(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
    #endif

#endif

class OSShim
{
public:
    void sleep(uint32_t ms);
    uint32_t millis();
    OSShim_Mutex* createMutex();

    void* malloc(uint32_t size);
    void free(void* ptr);

    typedef void (*SleepCallback)(uint32_t ms);
    typedef uint32_t (*MillisCallback)();
    typedef OSShim_Mutex* (*CreateMutexCallback)();
    typedef void* (*MallocCallback)(uint32_t size);
    typedef void (*FreeCallback)(void* ptr);

    OSShim(SleepCallback sleepCallback, MillisCallback millisCallback, CreateMutexCallback createMutexCallback, MallocCallback mallocCallback, FreeCallback freeCallback);

private:
    SleepCallback sleepCallback;
    MillisCallback millisCallback;
    CreateMutexCallback createMutexCallback;
    MallocCallback mallocCallback;
    FreeCallback freeCallback;
};

#endif // OSShim_h