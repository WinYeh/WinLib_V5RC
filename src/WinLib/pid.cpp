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
    if (windupRange == 0 || std::fabs(error) < windupRange) 
        integral += error;

    // Save each term so debugPID can read the breakdown later. The d_term must
    // use the OLD prevError, so compute all three before overwriting prevError.
    p_term = kP * error;
    i_term = kI * integral;
    d_term = kD * (error - prevError);
    prevError = error;

    return p_term + i_term + d_term;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
    p_term = 0;
    i_term = 0;
    d_term = 0;
}

void PID::setkP(float newkP) { this->kP = newkP; }
void PID::setkI(float newkI) { this->kI = newkI; }
void PID::setkD(float newkD) { this->kD = newkD; }
void PID::setWindupRange(float newWindupRange) { this->windupRange = newWindupRange; }

float PID::getkP() const { return kP; }
float PID::getkI() const { return kI; }
float PID::getkD() const { return kD; }
float PID::getWindupRange() const { return windupRange; }

float PID::getPTerm() const { return p_term; }
float PID::getITerm() const { return i_term; }
float PID::getDTerm() const { return d_term; }


/* linear PID */
linear_PID::linear_PID(float kP, float kI, float kD, float windupRange)
    : PID(kP, kI, kD, windupRange) {};

float linear_PID::compute(const float error)
{
    if (windupRange == 0 || std::fabs(error) < windupRange) integral += error;

    // linear PID shapes the P term through xEe_Func. Save all three terms
    // (d_term uses the old prevError, so compute before overwriting it).
    p_term = kP * xEe_Func(error);
    i_term = kI * integral;
    d_term = kD * (error - prevError);
    prevError = error;

    return p_term + i_term + d_term;
}


/* angular PID */
angular_PID::angular_PID(float kP, float kI, float kD, float windupRange)
    : PID(kP, kI, kD, windupRange) {};

float angular_PID::compute(const float error)
{
    if (windupRange == 0 || std::fabs(error) < windupRange) integral += error;

    // angular PID uses a custom shaped derivative; keep that math in a local,
    // then save the three terms (compute before overwriting prevError).
    int error_dir = std::fabs(error) / error;
    const float derivative = (error_dir * 5E-6 * xEe_Func(error * error_dir)) - (prevError - error);

    p_term = kP * error;
    i_term = kI * integral;
    d_term = kD * derivative;
    prevError = error;

    return p_term + i_term + d_term;
}


/* gain scheduling */
float WinLib::asymptoticGain(float setpoint, float initial, float final, float knee, float power)
{
    // |setpoint|^power — the curve's input. fabs so the turn/move direction
    // never matters; only the SIZE of the motion picks the gain.
    const float s = std::pow(std::fabs(setpoint), power);
    // knee > 0 keeps the denominator non-zero, so at setpoint 0 this cleanly
    // returns `initial`. (With knee == 0 it would be 0/0 at setpoint 0.)
    return (final - initial) * s / (s + std::pow(knee, power)) + initial;
}
