#ifndef OSShim_h
#define OSShim_h

#include <cstdint>

#ifdef ESP_PLATFORM
#warning "MOVE THIS CODE TO ESP PLATFORM IMPLEMENTATION"

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

    #ifndef OSShimWarning
    #define OSShimWarning(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
    #endif

    #ifndef OSShimError
    #define OSShimError(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
    #endif

#endif

#define OSShimVerbose(tag, format, ...) printf( "Verbose - %s: " format "\n", tag, ##__VA_ARGS__)
#define OSShimDebug(tag, format, ...) printf( "Debug - %s: " format "\n", tag, ##__VA_ARGS__)
#define OSShimInfo(tag, format, ...) printf( "Info - %s: " format "\n", tag, ##__VA_ARGS__)
#define OSShimWarning(tag, format, ...) printf( "Warning - %s: " format "\n", tag, ##__VA_ARGS__)
#define OSShimError(tag, format, ...) printf( "Error - %s: " format "\n", tag, ##__VA_ARGS__)

class OSShim_Mutex
{
public:
    virtual void signal() = 0;
    virtual bool wait(uint32_t max_time_to_wait_ms) = 0;
};

class OSShim
{
public:
    virtual void osSleep(uint32_t ms) = 0;
    virtual uint32_t osMillis() = 0;
    virtual OSShim_Mutex* osCreateMutex() = 0;

    virtual void* osMalloc(uint32_t size) = 0;
    virtual void osFree(void* ptr) = 0;

    virtual ~OSShim() = default;
};

#endif // OSShim_h