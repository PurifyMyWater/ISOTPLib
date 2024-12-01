#ifndef CANMESSAGEACKQUEUE_H
#define CANMESSAGEACKQUEUE_H

#include <list>
#include "N_USData_Runner.h"

class CANMessageACKQueue
{
public:
    void run_step();

private:
    std::list<N_USData_Runner> messageQueue;
};

#endif // CANMESSAGEACKQUEUE_H
