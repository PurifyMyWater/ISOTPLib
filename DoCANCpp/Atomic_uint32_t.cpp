#include "Atomic_uint32_t.h"

 Atomic_uint32_t::Atomic_uint32_t(uint32_t initialValue, OSShim& osShim)
{
    this->osShim = &osShim;
    uint32_value = initialValue;
    this->mutex = this->osShim->osCreateMutex();
}

bool Atomic_uint32_t::get(uint32_t* out, uint32_t timeout) const
{
    if (mutex->wait(timeout))
    {
        *out = uint32_value;
        mutex->signal();
        return true;
    }
    return false;
}

bool Atomic_uint32_t::set(uint32_t newValue, uint32_t timeout)
{
    if (mutex->wait(timeout))
    {
        uint32_value = newValue;
        mutex->signal();
        return true;
    }
    return false;
}

bool Atomic_uint32_t::add(uint32_t amount, uint32_t timeout)
{
    if (mutex->wait(timeout))
    {
        uint32_value += amount;
        mutex->signal();
        return true;
    }
    return false;
}

bool Atomic_uint32_t::sub(uint32_t amount, uint32_t timeout)
{
    if (mutex->wait(timeout))
    {
        uint32_value -= amount;
        mutex->signal();
        return true;
    }
    return false;
}

