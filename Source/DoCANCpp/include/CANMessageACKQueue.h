#ifndef CANMESSAGEACKQUEUE_H
#define CANMESSAGEACKQUEUE_H

#include <list>
#include "CANInterface.h"

class N_USData_Runner;

class CANMessageACKQueue
{
public:
    explicit CANMessageACKQueue(CANInterface& canInterface);

    void run_step();

    bool writeFrame(N_USData_Runner& runner, CANFrame& frame);

    constexpr static const char* TAG = "DoCANCpp-CANMessageACKQueue";

private:
    std::list<N_USData_Runner*> messageQueue;
    CANInterface* canInterface;
};

#endif // CANMESSAGEACKQUEUE_H
