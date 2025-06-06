#include "Timer_N.h"

Timer_N::Timer_N(OSInterface& osInterface)
{
    this->osInterface = &osInterface;
    elapsedTime       = 0;
    startTime         = 0;
    timerRunning      = false;
}

void Timer_N::stopTimer()
{
    timerRunning = false;
    elapsedTime += osInterface->osMillis() - startTime;
}

void Timer_N::startTimer()
{
    elapsedTime  = 0;
    startTime    = osInterface->osMillis();
    timerRunning = true;
}
void Timer_N::clearTimer()
{
    timerRunning = false;
    elapsedTime = 0;
}

bool Timer_N::isTimerRunning() const
{
    return timerRunning;
}

uint32_t Timer_N::getStartTimeStamp() const
{
    return startTime;
}

uint32_t Timer_N::getElapsedTime_ms() const
{
    return timerRunning ? osInterface->osMillis() - startTime : elapsedTime;
}
