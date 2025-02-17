#include "Timer_N.h"

Timer_N::Timer_N(OSInterface& osShim)
{
    this->osShim = &osShim;
    elapsedTime = 0;
    startTime = 0;
    isTimerRunning = false;
}

void Timer_N::stopTimer()
{
    isTimerRunning = false;
    elapsedTime += osShim->osMillis() - startTime;
}

void Timer_N::startTimer()
{
    elapsedTime = 0;
    startTime = osShim->osMillis();
    isTimerRunning = true;
}

uint32_t Timer_N::getStartTimeStamp() const { return startTime; }

uint32_t Timer_N::getElapsedTime_ms() const { return isTimerRunning ? osShim->osMillis() - startTime : elapsedTime; }
