#include "Atomic_int64_t.h"

Atomic_int64_t::Atomic_int64_t(const int64_t initialValue, OSInterface& OSInterface)
{
    this->osInterface = &OSInterface;
    internalValue     = initialValue;
    this->mutex       = this->osInterface->osCreateMutex();
}

bool Atomic_int64_t::get(int64_t* out, const uint32_t timeout) const
{
    if (mutex->wait(timeout))
    {
        *out = internalValue;
        mutex->signal();
        return true;
    }
    return false;
}

bool Atomic_int64_t::set(const int64_t newValue, const uint32_t timeout)
{
    if (mutex->wait(timeout))
    {
        internalValue = newValue;
        mutex->signal();
        return true;
    }
    return false;
}

bool Atomic_int64_t::add(const int64_t amount, const uint32_t timeout)
{
    if (mutex->wait(timeout))
    {
        internalValue += amount;
        mutex->signal();
        return true;
    }
    return false;
}

bool Atomic_int64_t::sub(const int64_t amount, const uint32_t timeout)
{
    if (mutex->wait(timeout))
    {
        internalValue -= amount;
        mutex->signal();
        return true;
    }
    return false;
}

bool Atomic_int64_t::subIfResIsGreaterThanZero(const int64_t amount, const uint32_t timeout)
{
    if (mutex->wait(timeout))
    {
        int64_t res = internalValue - amount;
        if (res > 0)
        {
            internalValue = res;
            mutex->signal();
            return true;
        }
        mutex->signal();
    }
    return false;
}
