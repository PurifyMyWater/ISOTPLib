#include "CANInterface.h"

#include <cstdio>

#include "../ISOTP/include/ISOTP_Common.h"

const char* N_TAtypeToString(const N_TAtype_t nTAtype)
{
    switch (nTAtype)
    {
        case N_TATYPE_5_CAN_CLASSIC_29bit_Physical:
            return "N_TATYPE_5_CAN_CLASSIC_29bit_Physical";
        case N_TATYPE_6_CAN_CLASSIC_29bit_Functional:
            return "N_TATYPE_6_CAN_CLASSIC_29bit_Functional";
        default:
            return "UNKNOWN_N_TATYPE";
    }
}

const char* nAiToString(const N_AI& nAi)
{
    static char buffer[MAX_N_AI_STR_SIZE]; // 72 = 40 (N_TAtype) + 3 (N_SA) + 3 (N_TA) + 25 (for the format string) + 1
                                           // (for the null terminator)

    snprintf(buffer, sizeof(buffer), "{N_SA=%u, N_TA=%u, N_TAtype=%s}", nAi.N_SA, nAi.N_TA,
             N_TAtypeToString(nAi.N_TAtype));

    return buffer;
}

const char* frameDataToString(const uint8_t* data, const uint8_t data_length_code)
{
    /* Allocate twice the number of bytes in the "buf" array because each byte would
     * be converted to two hex characters, also add an extra space for the terminating
     * null byte.
     */
    static char output[(CAN_FRAME_MAX_DLC * 2) + 1];
    const char  size = MIN(data_length_code, CAN_FRAME_MAX_DLC);

    char* ptr = &output[0];

    for (int i = 0; i < size; i++)
    {
        ptr += sprintf(ptr, "%02X", data[i]);
    }

    return output;
}

const char* frameToString(const CANFrame& frame)
{
    static char buffer[MAX_FRAME_STR_SIZE]; // 181 = 72 (N_AI) + 5 (flags) + 1 (data_length_code) + 17 (data) + 85
                                            // (format string) + 1 (null terminator)

    snprintf(buffer, sizeof(buffer),
             "{N_AI=%s, flags={extd=%u, rtr=%u, ss=%u, self=%u, dlc_non_comp=%u}, data_length_code=%u, data=[0x%s]}",
             nAiToString(frame.identifier), frame.extd, frame.rtr, frame.ss, frame.self, frame.dlc_non_comp,
             frame.data_length_code, frameDataToString(frame.data, frame.data_length_code));

    return buffer;
}

const char* CANInterface::ackResultToString(const ACKResult ackResult)
{
    switch (ackResult)
    {
        case ACK_SUCCESS:
            return "ACK_SUCCESS";
        case ACK_ERROR:
            return "ACK_ERROR";
        case ACK_NONE:
            return "ACK_NONE";
        default:
            return "UNKNOWN_ACK_RESULT";
    }
}
