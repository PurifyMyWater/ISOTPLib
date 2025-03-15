#ifndef TIMER_N_H
#define TIMER_N_H

#include "OSInterface.h"

class Timer_N
{
public:
    explicit Timer_N(OSInterface& osInterface);
    void stopTimer();
    void startTimer();

    [[nodiscard]] uint32_t getStartTimeStamp() const;
    [[nodiscard]] uint32_t getElapsedTime_ms() const;

private:
    OSInterface* osInterface;
    uint32_t     elapsedTime;
    uint32_t     startTime;
    bool         isTimerRunning;
};

#endif // TIMER_N_H
