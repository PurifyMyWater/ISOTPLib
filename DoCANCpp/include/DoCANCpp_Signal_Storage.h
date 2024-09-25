#ifndef CANMASTER_DOCANCPP_SIGNAL_STORAGE_H
#define CANMASTER_DOCANCPP_SIGNAL_STORAGE_H

#include "DoCANCpp.h"
#include "DoCANCpp_Utils.h"

/**
 * @brief Optional Class that stores the signals issued by DoCANCpp.
 * It is an upper layer of the DoCANCpp, and uses the callbacks to store the signals into buffers for later processing.
 */
class DoCANCpp_Signal_Storage
{
public:
    bool N_USData_confirm(N_AI* nAi, N_Result* nResult, Mtype mtype = Mtype_Diagnostics); // TODO this is a signal

    bool N_USData_indication(N_AI* nAi, uint8_t** messageData, uint32_t* length, N_Result* nResult, Mtype* mtype); // TODO this is a signal
    bool N_USData_FF_indication(N_AI* nAi, uint32_t* length, Mtype* mtype = nullptr); // TODO this is a signal

    DoCANCpp& getDoCANCpp();
};


#endif //CANMASTER_DOCANCPP_SIGNAL_STORAGE_H
