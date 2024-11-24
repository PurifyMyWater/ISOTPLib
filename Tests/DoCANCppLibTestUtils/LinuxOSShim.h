#ifndef DOCANCPPLIBTEST_OSLINUXSHIM_H
#define DOCANCPPLIBTEST_OSLINUXSHIM_H

#include "OSShim.h"

class LinuxOSShim : public OSShim
{
public:
    void osSleep(uint32_t ms) override;
    uint32_t osMillis() override;
    OSShim_Mutex* osCreateMutex() override;

    void* osMalloc(uint32_t size) override;
    void osFree(void* ptr) override;
};

#endif // DOCANCPPLIBTEST_OSLINUXSHIM_H
