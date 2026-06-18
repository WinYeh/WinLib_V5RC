#include "WinLib/debugPrint.hpp"
#include "pros/rtos.hpp"   // pros::millis for the timestamp
#include <cstdio>          // printf

using namespace WinLib;

// Default off: nothing prints until the user raises this (e.g. in autonomous()).
int WinLib::debugRefreshTime = 50;   // in ms

void WinLib::debugPID(const char* label, float error, float output, const PID& pid)
{
    printf("[%s] t=%lums err=%.2f out=%.2fV p_term=%.2f i_term=%.2f d_term=%.2f\n",
           label,
           (unsigned long)pros::millis(),
           error,
           output,
           pid.getPTerm(),
           pid.getITerm(),
           pid.getDTerm());
}

void WinLib::debugPose(const Pose& current, const Pose& target)
{
    printf("[pose] x=%.2f y=%.2f theta=%.2fdeg | tgt=(%.2f, %.2f) dist=%.2f\n",
           current.x,
           current.y,
           current.theta,
           target.x,
           target.y,
           current.distance(target));
}
