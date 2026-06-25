#include "WinLib/chassis/chassis.hpp"
#include "WinLib/chassis/Odom.hpp"
#include "WinLib/pid.hpp"
#include "WinLib/exitcondition.hpp"
#include "WinLib/timer.hpp"
#include "WinLib/util.hpp"
#include "WinLib/debugPrint.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

using namespace WinLib;

// turnBy — a RELATIVE in-place turn.
//
// turnToHeading faces an ABSOLUTE compass heading, so it works on the wrapped
// 0–360° heading and can never express a move bigger than half a turn (360° and
// 0° are the same direction). turnBy instead rotates a set NUMBER of degrees from
// wherever the robot is right now, measured against the UNBOUNDED accumulated
// heading (getPose().theta keeps counting past 360° and never wraps).
//
// Progress is tracked as a MAGNITUDE: error = angle − |theta − start|. That makes
// it SIGN-ROBUST — the robot converges no matter which physical direction it spins
// — and it handles a full revolution and beyond (360 = one turn, 720 = two).
//
//   `angle` must be POSITIVE. Because progress is a magnitude (≥ 0), a negative
//   angle can never be reached, so the motion would never finish. params.direction
//   is unused; the other AngularParams fields (maxSpeed / minSpeed / earlyExitRange)
//   still apply.
void Chassis::turnBy(float angle, int timeout, AngularParams params)
{
    // Build the exit condition + timer from the chassis's angular settings.
    // Fresh objects every call → clean integral / timer state.
    ExitCondition exit(angularSettings.exitRange,
                       angularSettings.exitTimeout);

    Timer timer(timeout);

    // Snapshot the unbounded start heading. Progress is how far we've rotated from
    // it (a magnitude, ≥ 0), and the target is simply `angle` degrees of rotation,
    // so error = angle − |theta − start| counts down to 0 over the turn.
    float start  = getPose(false).theta;   // degrees, unbounded
    float target = angle;

    float error = 0;

    // |angle| IS the size of this turn, so gain-schedule kP straight from it (same
    // idea as turnToHeading using its initial error). kP is fixed for the motion:
    // a big spin gets a gentle kP, a tiny nudge a snappy one. Falls back to the
    // constant kP when no schedule curve is configured.
    float initialError = angle;
    float kP = angularSettings.gains
        ? asymptoticGain(std::fabs(initialError),
                         angularSettings.gains->initial,
                         angularSettings.gains->final,
                         angularSettings.gains->knee,
                         angularSettings.gains->power)
        : angularSettings.kP;

    angular_PID pid(kP,
                    angularSettings.kI,
                    angularSettings.kD,
                    angularSettings.windupRange);

    // One-time scheduling readout (only when debug is on).
    if (debugRefreshTime > 0)
        printf("turnBy start: angle=%.1f target=%.1f kP=%.4f\n",
               angle, target, kP);

    // Debug throttle clock: lastDebug holds the time of the last print.
    uint32_t lastDebug = 0;

    // Main control loop, 100 Hz (same cadence as odom).
    while (!timer.isDone())
    {
        // curr = how far we've rotated from start (a magnitude). error counts down
        // to 0 as that reaches `angle`. NO angleError, NO wrap — turnBy's whole point.
        float curr = fabs(getPose(false).theta - start);
        error = target - (curr * sgn(target) );

        // Feed |error| to the exit condition and check it.
        //  - exit fires after |error| stays inside exitRange for exitTimeout ms.
        //  - earlyExitRange is an instantaneous "close enough" threshold, only
        //    meaningful when minSpeed > 0.
        exit.update(error);

        if (exit.getExit())
            break;
        if (std::fabs(error) < std::fabs(params.earlyExitRange))
            break;

        // PID output, in volts.
        float output = pid.compute(error);

        // Clamp to [-maxSpeed, +maxSpeed] (volts; 12 V hardware ceiling).
        output = clamp(output, params.maxSpeed, -params.maxSpeed);

        // Apply the min-speed floor.
        if (std::fabs(output) < std::fabs(params.minSpeed))
        {
            output = params.minSpeed * sgn(output);
        }

        // Mix for in-place rotation. Positive output increases heading, the same
        // convention turnToHeading uses, so a positive `angle` turns that way.
        // Chassis::move_voltage signature is (left, right).
        move_voltage(+output, -output);

        // Print telemetry at most once every debugRefreshTime ms (0 = off).
        if (debugRefreshTime > 0 &&
            pros::millis() - lastDebug >= (uint32_t)debugRefreshTime)
        {
            debugPID("turnBy", error, output, pid);
            printf("theta: %.2f\n", getPose(false).theta);
            lastDebug = pros::millis();
        }

        pros::delay(10);
    }

    // Stop the motors. Stopping behavior (coast/brake/hold) is whatever the caller
    // set via Chassis::setBrakeMode.
    move_voltage(0, 0);
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    printf("turnBy (%.1f) done, error = %.2f, heading = %.2f---\n",
           angle, error, getHeading());
    printf("batteryLevel: %.0f\n", pros::c::battery_get_capacity());
}
