#include <cmath>
#include "pros/rtos.hpp"
#include "WinLib/exitcondition.hpp"
#include "WinLib/util.hpp" // IWYU pragma: keep (for WinLib::degToRad)
using namespace WinLib;

ExitCondition::ExitCondition(const float range, const int time)
    // range is a magnitude (an |error| window), so force it positive.
    : range(std::fabs(range)),
      time(time) {}

bool ExitCondition::getExit() 
{ 
    return done; 
}

bool ExitCondition::update(const float input) 
{
    const int curTime = pros::millis();
    if (std::fabs(input) > range) 
        startTime = -1;
    else if (startTime == -1) 
        startTime = curTime;
    else if (curTime >= startTime + time) 
        done = true;
    return done;
}

void ExitCondition::reset()
{
    startTime = -1;
    done = false;
}

void ExitCondition::setRange(const float newRange)
{
    range = std::fabs(newRange);   // range is a magnitude — keep it positive
    reset();
}

void ExitCondition::setTime(const int newTime)
{
    time = newTime;
    reset();
}

float ExitCondition::getRange() const 
{ 
    return range; 
}
int ExitCondition::getTime()  const 
{ 
    return time; 
}

bool hasReachedMinVel(float velocity, float minVelocity)
{
    return std::fabs(velocity) <= std::fabs(minVelocity);
}

bool hasCrossedTarget(Pose pose, Pose target, float theta, float tolerance = 0)
{
    const float heading = WinLib::DegToRad(theta);
    return (pose.y - target.y) * -std::cos(heading) >= std::sin(heading) * (pose.x - target.x) + tolerance;
    
}