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

void Chassis::turnToHeading(float theta, int timeout, AngularParams params)
{
    // Build the exit condition + timer from the chassis's angular settings.
    // Fresh objects every call → clean integral / timer state.
    ExitCondition exit(angularSettings.exitRange,
                       angularSettings.exitTimeout);

    Timer timer(timeout);

    float error = 0;
    // Pre-wrap the target into [0, 360°) so a caller passing -30 or 720 still
    // produces the same intended heading. fmod can return negative, so add
    // 360 if it does.
    float target = std::fmod(theta, 360.0f);
    if (target < 0) target += 360.0f;

    // The initial heading error is the "size" of this turn. If angular settings
    // carry a gain-schedule curve, pick kP from that size ONCE here (a big swing
    // gets a gentle kP, a tiny correction a snappy one); otherwise fall back to
    // the plain constant kP. Either way kP is fixed for the whole motion.
    float initialError = angleError(target, getHeading(), /*radians=*/false, params.direction);
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
        printf("turnToHeading start: target=%.1f initErr=%.2f kP=%.4f\n",
               target, initialError, kP);

    // Debug throttle clock (T1): the motion owns the timing, so the whole
    // mechanism is visible. lastDebug holds the time of the last print.
    uint32_t lastDebug = 0;

    // Main control loop, 100 Hz (same cadence as odom).
    while (!timer.isDone())
    {
        // Current heading, already wrapped into [0, 360°) by getHeading().
        float curr = getHeading();

        // Signed angular error.
        // AUTO → shortest-path value in [-180°, +180°].
        // CW_CLOCKWISE / CCW_COUNTERCLOCKWISE → forced-direction error.
        error = angleError(target, curr, /*radians=*/false, params.direction);

        // Feed |error| to the exit condition and check it.
        //  - exit fires after |error| stays inside exitRange for exitTimeout ms.
        //  - earlyExitRange is an instantaneous "close enough" threshold,
        //    only meaningful when minSpeed > 0 (since with no floor the PID
        //    would keep pushing past it anyway).
        exit.update(std::fabs(error));

        if (exit.getExit())
            break;
        if (std::fabs(error) < params.earlyExitRange)
            break;

        // PID output, in volts.
        float output = pid.compute(error);        

        // Clamp to [-maxSpeed, +maxSpeed]. maxSpeed is in volts; the
        // hardware ceiling is 12 V, but the caller may want a slower turn.
        output = clamp(output, params.maxSpeed, -params.maxSpeed);

        // Apply the min-speed floor.
        if (std::fabs(output) < std::fabs(params.minSpeed) )
        {
            output = (output > 0) ? params.minSpeed : -params.minSpeed;
        }

        // Mix for in-place rotation (see sign convention at top of file).
        // Chassis::move_voltage signature is (left, right).
        move_voltage(+output, -output);

        // Print telemetry at most once every debugRefreshTime ms (0 = off).
        // out here is the post-clamp/floor voltage actually sent to the motors,
        // so during saturation it won't equal p_term + i_term + d_term — that
        // gap is your saturation signal.
        if (debugRefreshTime > 0 &&
            pros::millis() - lastDebug >= (uint32_t)debugRefreshTime)
        {
            debugPID("ang", error, output, pid);
            lastDebug = pros::millis();
        }

        pros::delay(10);
    }

    // Stop the motors. The actual stopping behavior (coast vs brake vs
    // hold) is whatever the caller set via Chassis::setBrakeMode.
    move_voltage(0, 0);
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
   
    printf ("turnToHeading (%.1f) done, error = %.2f, imu = %.2f---\n", target, error, getPose(false).theta);
    printf ("batteryLevel: %.0f\n", pros::c::battery_get_capacity());
}
