#ifndef ATOMIC_UINT32_H
#define ATOMIC_UINT32_H

#include <cstdint>
#include "OSShim.h"

constexpr uint32_t DEFAULT_Atomic_int64_t_TIMEOUT = 100;

class Atomic_int64_t
{
public:
    Atomic_int64_t(int64_t initialValue, OSShim& OSShim);
    bool get(int64_t* out, uint32_t timeout = 100) const;
    bool set(int64_t newValue, uint32_t timeout = DEFAULT_Atomic_int64_t_TIMEOUT);
    bool add(int64_t amount, uint32_t timeout = DEFAULT_Atomic_int64_t_TIMEOUT);
    bool sub(int64_t amount, uint32_t timeout = DEFAULT_Atomic_int64_t_TIMEOUT);
    bool subIfResIsGreaterThanZero(int64_t amount, uint32_t timeout = DEFAULT_Atomic_int64_t_TIMEOUT);

private:
    int64_t internalValue;
    OSShim* osShim;
    OSShim_Mutex* mutex;
};

#endif // ATOMIC_UINT32_H
