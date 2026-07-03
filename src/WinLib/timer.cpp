#include "pros/rtos.hpp"    // IWYU pragma: keep
#include "WinLib/timer.hpp"

using namespace WinLib;

Timer::Timer(uint32_t time)
    : timeout(time) 
{
    lastTime = pros::millis();
}

void Timer::update()
{
    const uint32_t time = pros::millis(); // get time from RTOS
    if (!paused) timeWaited += time - lastTime; // don't update if paused
    lastTime = time; // update last time
}

uint32_t Timer::getTimeout() 
{
    this->update(); // update timer variables 
    return timeout;
}

uint32_t Timer::getTimeLeft() 
{
    this->update(); // update timer variables
    const int delta = timeout - timeWaited; // calculate how much time is left
    return (delta > 0) ? delta : 0; // return 0 if timer is done
}

uint32_t Timer::getTimePassed() 
{
    this->update(); // update timer variables
    return timeWaited;
}

bool Timer::isDone() 
{
    this->update(); // update timer variables
    const int delta = timeout - timeWaited; // calculate how much time is left
    return delta <= 0;
}

bool Timer::isPaused() 
{
    this->update(); // update timer variables
    return paused;
}

void Timer::set(uint32_t time) 
{
    timeout = time; // set how long to wait
    reset();
}

void Timer::reset() 
{
    timeWaited = 0;
    lastTime = pros::millis();
}

void Timer::pause() 
{
    if (!paused) lastTime = pros::millis();
    paused = true;
}

void Timer::resume() 
{
    if (paused) lastTime = pros::millis();
    paused = false;
}

void Timer::waitUntilDone() 
{
    do pros::delay(5);
    while (!this->isDone());
}
