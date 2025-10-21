#ifndef ATOMIC_H
#define ATOMIC_H

#include "OSInterface.h"

constexpr uint32_t DEFAULT_Atomic_TIMEOUT_ms = 100;

template <typename Type> class Atomic
{
public:
    Atomic(Type initialValue, OSInterface_Mutex* OSInterfaceMutex)
    {
        internalValue     = initialValue;
        this->mutex       = OSInterfaceMutex;
        this->fromOSInterface = false;
    }

    Atomic(Type initialValue, OSInterface& OSInterface)
    {
        internalValue     = initialValue;
        this->mutex       = OSInterface.osCreateMutex();
        this->fromOSInterface = true;
    }
    
    ~Atomic()
    {
        if (fromOSInterface && mutex != nullptr)
        {
            delete mutex;
        }
    }

    bool get(Type* out, uint32_t timeout = DEFAULT_Atomic_TIMEOUT_ms) const
    {
        if (mutex != nullptr && mutex->wait(timeout))
        {
            *out = internalValue;
            mutex->signal();
            return true;
        }
        return false;
    }

    bool set(Type newValue, uint32_t timeout = DEFAULT_Atomic_TIMEOUT_ms)
    {
        if (mutex != nullptr && mutex->wait(timeout))
        {
            internalValue = newValue;
            mutex->signal();
            return true;
        }
        return false;
    }

    bool add(Type amount, uint32_t timeout = DEFAULT_Atomic_TIMEOUT_ms)
    {
        if (mutex != nullptr && mutex->wait(timeout))
        {
            internalValue += amount;
            mutex->signal();
            return true;
        }
        return false;
    }

    bool sub(Type amount, uint32_t timeout = DEFAULT_Atomic_TIMEOUT_ms)
    {
        if (mutex != nullptr && mutex->wait(timeout))
        {
            internalValue -= amount;
            mutex->signal();
            return true;
        }
        return false;
    }

    bool subIfResIsGreaterThanZero(Type amount, uint32_t timeout = DEFAULT_Atomic_TIMEOUT_ms)
    {
        if (mutex != nullptr && mutex->wait(timeout))
        {
            Type res = internalValue - amount;
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

private:
    Type               internalValue;
    OSInterface_Mutex* mutex;
    bool fromOSInterface = false;
};

#endif // ATOMIC_H
