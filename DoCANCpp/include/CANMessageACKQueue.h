#ifndef CANMESSAGEACKQUEUE_H
#define CANMESSAGEACKQUEUE_H

#include <list>
#include "CANShim.h"

class N_USData_Runner;

class CANMessageACKQueue
{
public:
    explicit CANMessageACKQueue(CANShim& canShim);

    void run_step();

    bool writeFrame(N_USData_Runner& runner, CANFrame& frame);

private:
    std::list<N_USData_Runner*> messageQueue;
    CANShim* canShim;
};

#endif // CANMESSAGEACKQUEUE_H
