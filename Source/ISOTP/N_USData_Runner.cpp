#include "N_USData_Runner.h"

const char* N_USData_Runner::runnerTypeToString(RunnerType type)
{
    switch (type)
    {
        case RunnerUnknownType:
            return "Unknown Runner Type";
        case RunnerRequestType:
            return "Request Runner Type";
        case RunnerIndicationType:
            return "Indication Runner Type";
        default:
            return "Invalid Runner Type";
    }
}

const char* N_USData_Runner::frameCodeToString(FrameCode code)
{
    switch (code)
    {
        case SF_CODE:
            return "SF";
        case FF_CODE:
            return "FF";
        case CF_CODE:
            return "CF";
        case FC_CODE:
            return "FC";
        default:
            return "Unknown Frame Code";
    }
}

const char* N_USData_Runner::flowStatusToString(FlowStatus status)
{
    switch (status)
    {
        case CONTINUE_TO_SEND:
            return "Continue to Send";
        case WAIT:
            return "Wait";
        case OVERFLOW:
            return "Overflow";
        case INVALID_FS:
            return "Invalid Flow Status";
        default:
            return "Unknown Flow Status";
    }
}
