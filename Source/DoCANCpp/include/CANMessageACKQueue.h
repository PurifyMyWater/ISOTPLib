#ifndef CANMESSAGEACKQUEUE_H
#define CANMESSAGEACKQUEUE_H

#include <list>
#include "CANInterface.h"

class N_USData_Runner;

class CANMessageACKQueue
{
public:
    explicit CANMessageACKQueue(CANInterface& canShim);

    void run_step();

    bool writeFrame(N_USData_Runner& runner, CANFrame& frame);

private:
    std::list<N_USData_Runner*> messageQueue;
    CANInterface* canShim;
};

#endif // CANMESSAGEACKQUEUE_H
