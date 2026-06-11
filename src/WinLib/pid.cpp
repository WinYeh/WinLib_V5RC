#include "WinLib/pid.hpp"
#include "WinLib/util.hpp"
#include <cmath>

using namespace WinLib;

PID::PID(float kP, float kI, float kD, float windupRange)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange) {}

float PID::compute(const float error)
{
    // anti-windup: only accumulate integral when close enough to target.
    // windupRange == 0 disables the check (always accumulate).
    if (windupRange == 0 || std::fabs(error) < windupRange) integral += error;

    // calculate derivative
    const float derivative = error - prevError;
    prevError = error;

    // calculate output
    return kP * error + kI * integral + kD * derivative;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}

void PID::setkP(float newkP) { this->kP = newkP; }
void PID::setkI(float newkI) { this->kI = newkI; }
void PID::setkD(float newkD) { this->kD = newkD; }
void PID::setWindupRange(float newWindupRange) { this->windupRange = newWindupRange; }

float PID::getkP() const { return kP; }
float PID::getkI() const { return kI; }
float PID::getkD() const { return kD; }
float PID::getWindupRange() const { return windupRange; }


/* linear PID */
linear_PID::linear_PID(float kP, float kI, float kD, float windupRange)
    : PID(kP, kI, kD, windupRange) {};

float linear_PID::compute(const float error)
{
    if (windupRange == 0 || std::fabs(error) < windupRange) integral += error;

    const float derivative = error - prevError;
    prevError = error;

    return kP * xEe_Func(error) + kI * integral + kD * derivative;
}


/* angular PID */
angular_PID::angular_PID(float kP, float kI, float kD, float windupRange)
    : PID(kP, kI, kD, windupRange) {};

float angular_PID::compute(const float error)
{
    if (windupRange == 0 || std::fabs(error) < windupRange) integral += error;

    // calculate derivative
    int error_dir = std::fabs(error) / error;
    const float derivative = (error_dir * 5E-6 * xEe_Func(error * error_dir)) - (prevError - error);
    prevError = error;

    // calculate output
    return kP * error + kI * integral + kD * derivative;
}
