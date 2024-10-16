#ifndef DOCANCPPLIBTEST_OSLINUXSHIM_H
#define DOCANCPPLIBTEST_OSLINUXSHIM_H

#include "OSShim.h"

class LinuxOSShim : public OSShim
{
public:
    void osSleep(uint32_t ms);
    uint32_t osMillis();
    OSShim_Mutex* osCreateMutex();

    void* osMalloc(uint32_t size);
    void osFree(void* ptr);

};

#endif //DOCANCPPLIBTEST_OSLINUXSHIM_H
