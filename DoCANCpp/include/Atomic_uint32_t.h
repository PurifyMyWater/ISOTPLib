#ifndef ATOMIC_UINT32_H
#define ATOMIC_UINT32_H

#include <cstdint>
#include "OSShim.h"

constexpr uint32_t DEFAULT_ATOMIC_UINT32_T_TIMEOUT = 100;

class Atomic_uint32_t
{
public:
    Atomic_uint32_t(uint32_t initialValue, OSShim& OSShim);
    bool get(uint32_t* out, uint32_t timeout = 100) const;
    bool set(uint32_t newValue, uint32_t timeout = DEFAULT_ATOMIC_UINT32_T_TIMEOUT);
    bool add(uint32_t amount, uint32_t timeout = DEFAULT_ATOMIC_UINT32_T_TIMEOUT);
    bool sub(uint32_t amount, uint32_t timeout = DEFAULT_ATOMIC_UINT32_T_TIMEOUT);
private:
    uint32_t uint32_value;
    OSShim* osShim;
    OSShim_Mutex* mutex;
};

#endif //ATOMIC_UINT32_H
