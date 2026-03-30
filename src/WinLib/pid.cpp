#include "WinLib/pid.hpp"
#include "WinLib/util.hpp"
#include <cmath>

using namespace WinLib;

PID::PID(float kP, float kI, float kD)
    : kP(kP), 
      kI(kI), 
      kD(kD) {}

float PID::compute(const float error) {
    // calculate integral
    integral += error;

    // calculate derivative
    const float derivative = error - prevError;
    prevError = error;

    // calculate output
    return kP * error + kI * integral + kD * derivative;
}

float PID::compute_l(const float error) {
    // calculate integral
    integral += error;

    // calculate derivative
    const float derivative = error - prevError;
    prevError = error;

    // calculate output
    return kP * xEe_Func(error) + kI * integral + kD * derivative;
}

float PID::compute_a(const float error) {
    // calculate integral
    integral += error;

    // calculate derivative
    int error_dir = fabs(error)/error;
    const float derivative =  (error_dir * 5E-6 * xEe_Func(error * error_dir) ) - (prevError - error) ;
    prevError = error;

    // calculate output
    return kP * error + kI * integral + kD * derivative;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}

void PID::setkP(float newkP)
{
    this->kP = newkP;
}
void PID::setkI(float newkI)
{
    this->kI = newkI;
}
void PID::setkD(float newkD)
{
    this->kD = newkD;
}