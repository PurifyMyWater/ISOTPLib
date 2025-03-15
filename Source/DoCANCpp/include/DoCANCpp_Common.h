#ifndef DOCANCPP_DATA_STRUCTURES_H
#define DOCANCPP_DATA_STRUCTURES_H

#define DOCANCPP_DISABLE_TIMEOUTS false

#ifdef NDEBUG
#define ASSERT_SAFE(expression, condition) expression
#else
#define ASSERT_SAFE(expression, condition) assert(expression condition)
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#include <cstdint>

constexpr uint32_t DoCANCpp_MaxTimeToWaitForSync_MS = 100;

constexpr uint8_t MAX_STMIN_MS_VALUE = 0x7F;
constexpr uint8_t MIN_STMIN_US_VALUE = 0xF1;
constexpr uint8_t MAX_STMIN_US_VALUE = 0xF9;

using STminUnit = enum SeparationTimeMinUnit { ms, usX100 };

using STmin = struct SeparationTimeMin
{
    uint8_t value;
    STminUnit unit;
};

using Mtype = enum Mtype { Mtype_Diagnostics, Mtype_Unknown };

using N_Result = enum N_Result {
    NOT_STARTED = 0,
    IN_PROGRESS_FF, // Only used by N_USData_Indication_Runner to indicate that the FF was received in this step.
    IN_PROGRESS,
    N_OK,
    N_TIMEOUT_A,
    N_TIMEOUT_Bs,
    N_TIMEOUT_Cr,
    N_WRONG_SN,
    N_INVALID_FS,
    N_UNEXP_PDU,
    N_WFT_OVRN,
    N_BUFFER_OVFLW,
    N_ERROR
};

const char* N_ResultToString(N_Result result);

#endif // DOCANCPP_DATA_STRUCTURES_H
