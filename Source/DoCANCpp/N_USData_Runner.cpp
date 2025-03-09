#include "N_USData_Runner.h"

#include <cassert>

N_USData_Runner::N_USData_Runner() { this->TAG = nullptr; }

uint32_t N_USData_Runner::getStMinInMs(STmin stMin)
{
    return stMin.unit == usX100 ? 1 : stMin.value; // 1 ms is the smallest resolution we can get.
}
