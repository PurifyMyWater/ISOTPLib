#ifndef DoCANCpp_H
#define DoCANCpp_H

#include <unordered_map>
#include <unordered_set>
#include <list>

#include "OSShim.h"
#include "CANShim.h"
#include "DoCANCpp_Data_Structures.h"
#include "N_USData_Runner.h"

#define DoCANCpp_N_AI_CONFIG(_N_TAtype, _N_TA, _N_SA) {.N_NFA_Header=0b110, .N_NFA_Padding=0b00, .N_TAtype=(_N_TAtype), .N_TA=(_N_TA), .N_SA=(_N_SA)}

typedef enum SeparationTimeMinUnit {ms, us} STminUnit;
typedef struct SeparationTimeMin {uint16_t value; STminUnit unit;} STmin;

constexpr uint32_t DoCANCpp_RunPeriod_MS = 100;
constexpr uint32_t DoCANCpp_MaxTimeToWaitForSync_MS = 100;
constexpr STmin DoCANCpp_DefaultSTmin = {20, ms};
constexpr uint8_t DoCANCpp_DefaultBlockSize = 0; // 0 means that all CFs are sent without waiting for an FC.

/**
 * This function is used to confirm the reception of a message.
 * @param nAi The N_AI of the message.
 * @param nResult The result of the reception.
 * @param mtype The Mtype of the message.
 */
typedef void (*N_USData_confirm_cb_t)(N_AI nAi, N_Result nResult, Mtype mtype);

/**
 * This function is used to indicate the reception of a message.
 * @warning The messageData is only valid during the callback, if you need to keep the data, copy it elsewhere.
 * @param nAi The N_AI of the message.
 * @param messageData The message data of the message.
 * @param messageLength The length of the message data.
 * @param nResult The result of the reception.
 * @param mtype The Mtype of the message.
 */
typedef void (*N_USData_indication_cb_t)(N_AI nAi, const uint8_t* messageData, uint32_t messageLength, N_Result nResult, Mtype mtype);

/**
 * This function is used to indicate the reception of the first frame of a multi-frame message.
 * @param nAi The N_AI of the message.
 * @param messageLength The expected length of the message.
 * @param mtype The Mtype of the message.
 */
typedef void (*N_USData_FF_indication_cb_t)(N_AI nAi, uint32_t messageLength, Mtype mtype);

/**
 * This class provides a C++ implementation of the DoCAN protocol aka ISO-TP, it currently only supports N_TAtype #5 & #6
 * (Standard CAN, 29bit ID Physical & Functional address modes using normal fixed addressing (See ISO 15765-2 for more details)).
 */
class DoCANCpp
{
public:
    static const char* TAG;

    /**
     * This function is used to request a message to be sent.
     * The message will be sent as soon as possible, but there is no guarantee on the timing.
     * @param nTa The N_TA to send the message to.
     * @param nTaType The N_TAtype of the N_TA.
     * @param messageData The message data to send.
     * @param length The length of the message data.
     * @param mtype The Mtype of the message.
     */
    void N_USData_request(typeof(N_AI::N_TA) nTa, N_TAtype_t nTaType, uint8_t* messageData, uint32_t length, Mtype mtype = Mtype_Diagnostics);

    /**
     * This function is used to run the DoCAN service.
     * It needs to be called periodically to allow the DoCAN service to run.
     * There are no limitations on the frequency of this function, timing is handled internally.
     * @param self The DoCANCpp object to run.
     */
    static void run_step(DoCANCpp* self);

    /**
     * This function is used to get the N_SA for this DoCANCpp object.
     * @return The N_SA for this DoCANCpp object.
     */
    typeof(N_AI::N_SA) getN_SA() const;

    /**
     * This function is used to set the N_SA for this DoCANCpp object.
     * From this point on, all messages sent or received by this object will have this N_SA.
     * Messages that are being received or sent before setting the new N_SA will not be affected and have the old N_SA.
     * @param nSA The N_SA to set for this DoCANCpp object.
     */
    void setN_SA(typeof(N_AI::N_SA) nSA);

    /**
     * This function is used to add a N_TA into the functional accepted N_TAs for this DoCANCpp object.
     * From this point on, all messages sent or received by this object will have this N_TA.
     * Messages that are being received or sent before setting the new N_TA will not be affected and have the old N_TA.
     * @param nTA The N_TA to set for this DoCANCpp object.
     */
    void addAcceptedFunctionalN_TA(typeof(N_AI::N_TA) nTA);

    /**
     * This function is used to remove a N_TA from the functional accepted N_TAs for this DoCANCpp object.
     * From this point on, all messages sent or received by this object will not have this N_TA.
     * Messages that are being received or sent before removing the N_TA will not be affected and have the old N_TA.
     * @param nTA The N_TA to remove for this DoCANCpp object.
     * @return True if the N_TA was removed, false otherwise.
     */
    bool removeAcceptedFunctionalN_TA(typeof(N_AI::N_TA) nTA);

    /**
     * This function is used to check if a N_TA is in the functional accepted N_TAs for this DoCANCpp object.
     * @param nTA The N_TA to check for in this DoCANCpp object.
     * @return True if the N_TA is in the functional accepted N_TAs, false otherwise.
     */
    bool hasAcceptedFunctionalN_TA(typeof(N_AI::N_TA) nTA);

    /**
     * This function is used to get the block size for this DoCANCpp object.
     * @return The block size for this DoCANCpp object.
     */
    uint8_t getBlockSize() const;

    /**
     * This function is used to set the block size for this DoCANCpp object.
     * All messages sent or received by this object will have this block size, even if they are being sent or received before setting the new block size.
     * @param blockSize The block size to set for this DoCANCpp object.
     */
    void setBlockSize(uint8_t blockSize);

    /**
     * This function is used to get the separation time for this DoCANCpp object.
     * @return The separation time for this DoCANCpp object.
     */
    STmin getSTmin() const;

    /**
     * This function is used to set the separation time for this DoCANCpp object.
     * All messages sent or received by this object will have this separation time, even if they are being sent or received before setting the new separation time.
     * @param stMin The separation time to set for this DoCANCpp object.
     * @return True if the separation time was set, false otherwise.
     */
    bool setSTmin(STmin stMin);

    DoCANCpp(typeof(N_AI::N_SA) nSA, uint32_t totalAvailableMemoryForRunners, N_USData_confirm_cb_t N_USData_confirm_cb, N_USData_indication_cb_t N_USData_indication_cb, N_USData_FF_indication_cb_t N_USData_FF_indication_cb, OSShim& osShim, CANShim& canShim, uint8_t blockSize = DoCANCpp_DefaultBlockSize, STmin stMin = DoCANCpp_DefaultSTmin);
private:

    // Shims
    OSShim* osShim;
    CANShim* canShim;

    // Synchronization & mutual exclusion
    OSShim_Mutex* volatile configMutex;
    OSShim_Mutex* volatile notStartedRunnersMutex;

    // Internal configuration (constant)
    N_USData_confirm_cb_t N_USData_confirm_cb;
    N_USData_indication_cb_t N_USData_indication_cb;
    N_USData_FF_indication_cb_t N_USData_FF_indication_cb;
    uint32_t availableMemoryForRunners;

    // Internal configuration (mutable)
    volatile typeof(N_AI::N_SA) nSA;
    std::unordered_set<typeof(N_AI::N_TA)> acceptedFunctionalN_TAs;
    uint8_t blockSize;
    STmin stMin{};

    // Internal data
    uint32_t lastRunTime;
    std::list<N_USData_Runner*> notStartedRunners;
    std::unordered_map<typeof(N_AI::N_AI), N_USData_Runner*> activeRunners;
    std::list<N_USData_Runner*> finishedRunners;
};

#endif // DoCANCpp_H
