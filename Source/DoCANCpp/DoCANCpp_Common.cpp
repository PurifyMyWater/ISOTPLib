#include "DoCANCpp_Common.h"

const char* N_ResultToString(const N_Result result)
{
    switch (result)
    {
        case NOT_STARTED:
            return "NOT_STARTED";
        case IN_PROGRESS:
            return "IN_PROGRESS";
        case N_OK:
            return "N_OK";
        case N_TIMEOUT_A:
            return "N_TIMEOUT_A";
        case N_TIMEOUT_Bs:
            return "N_TIMEOUT_Bs";
        case N_TIMEOUT_Cr:
            return "N_TIMEOUT_Cr";
        case N_WRONG_SN:
            return "N_WRONG_SN";
        case N_INVALID_FS:
            return "N_INVALID_FS";
        case N_UNEXP_PDU:
            return "N_UNEXP_PDU";
        case N_WFT_OVRN:
            return "N_WFT_OVRN";
        case N_BUFFER_OVFLW:
            return "N_BUFFER_OVFLW";
        case N_ERROR:
            return "N_ERROR";
        default:
            return "UNKNOWN";
    }
}

uint32_t getStMinInMs(const STmin stMin)
{
    if (stMin.value == 0)
    {
        // 0 ms is the smallest resolution we can get.
        // // And 0 usX100 is an invalid value, so no need to check it.
        return 0;
    }
    return stMin.unit == usX100 ? 1 : stMin.value; // 1 ms is the smallest resolution we can get in our implementation.
}
