#ifndef CANMESSAGEACKQUEUE_H
#define CANMESSAGEACKQUEUE_H

#include <list>
#include "CANInterface.h"
#include "OSInterface.h"

class N_USData_Runner;

class CANMessageACKQueue
{
public:
    explicit CANMessageACKQueue(CANInterface& canInterface, OSInterface& osInterface, const char* tag = TAG);
    ~CANMessageACKQueue();

    void runStep();

    void runAvailableAckCallbacks();

    bool writeFrame(N_USData_Runner& runner, CANFrame& frame);

    bool removeFromQueue(N_AI runnerNAi);

    constexpr static const char* TAG = "ISOTP-CANMessageACKQueue";

private:
    bool runNextAvailableAckCallback();
    void saveAck(ACKResult ack);

    const char*                                                     tag;
    OSInterface_Mutex*                                              mutex;
    std::list<std::pair<N_USData_Runner*, ACKResult>> messageQueue;
    CANInterface*                                                   canInterface;
};

#endif // CANMESSAGEACKQUEUE_H
