#include "Timer_N.h"

Timer_N::Timer_N(OSInterface& osInterface)
{
    this->osInterface = &osInterface;
    elapsedTime = 0;
    startTime = 0;
    isTimerRunning = false;
}

void Timer_N::stopTimer()
{
    isTimerRunning = false;
    elapsedTime += osInterface->osMillis() - startTime;
}

void Timer_N::startTimer()
{
    elapsedTime = 0;
    startTime = osInterface->osMillis();
    isTimerRunning = true;
}

uint32_t Timer_N::getStartTimeStamp() const { return startTime; }

uint32_t Timer_N::getElapsedTime_ms() const { return isTimerRunning ? osInterface->osMillis() - startTime : elapsedTime; }
