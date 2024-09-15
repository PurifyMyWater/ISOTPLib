#ifndef CANMASTER_DOCANCPP_DATA_STRUCTURES_H
#define CANMASTER_DOCANCPP_DATA_STRUCTURES_H

#include <cstdint>
#include <optional>
#include <iterator>
#include "CANShim.h"

typedef enum Mtype
{
    Mtype_Diagnostics,
    Mtype_Unknown
} Mtype;

typedef enum N_Result
{
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
} N_Result;

const char* N_Result_to_string(N_Result result);

#endif //CANMASTER_DOCANCPP_DATA_STRUCTURES_H
