#include "WinLib/chassis/chassis.hpp"
#include "WinLib/pid.hpp"
#include "WinLib/exitcondition.hpp"
#include "WinLib/timer.hpp"
#include "WinLib/util.hpp"
#include "WinLib/debugPrint.hpp"
#include "config.h"
#include "pros/rtos.hpp"
#include <cmath>

using namespace WinLib;

void Chassis::moveFor(float distance, int timeout, LateralParams params)
{
    // Build the exit condition + timer from the chassis's lateral settings.
    // Fresh objects every call → clean integral / timer state.
    ExitCondition exit(lateralSettings.exitRange,
                       lateralSettings.exitTimeout);

    Timer timer(timeout);

    float error = 0;
    // Store converted distance in mm to motor's degree as target for the PID controller
    float target = MMTodeg(distance); // in degrees

    // The commanded travel (|target|, in motor degrees) is the "size" of this
    // move. If lateral settings carry a gain-schedule curve, pick kP from that
    // size ONCE here; otherwise fall back to the plain constant kP. Either way
    // kP is fixed for the whole motion.
    float kP = lateralSettings.gains
        ? asymptoticGain(std::fabs(target),
                         lateralSettings.gains->initial,
                         lateralSettings.gains->final,
                         lateralSettings.gains->knee,
                         lateralSettings.gains->power)
        : lateralSettings.kP;

    PID pid(kP,
            lateralSettings.kI,
            lateralSettings.kD,
            lateralSettings.windupRange);

    // One-time scheduling readout (only when debug is on).
    if (debugRefreshTime > 0)
        printf("moveFor start: target=%.2f kP=%.5f\n", target, kP);

    // Debug throttle clock (T1): the motion owns the timing, so the whole
    // mechanism is visible. lastDebug holds the time of the last print.
    uint32_t lastDebug = 0;

    // Main control loop, 100 Hz (same cadence as odom).
    while (!timer.isDone())
    {
        // Current distance. Average the two sides of the chhasis to get a single distance reading, in degrees. 
        float curr = 0.5 * (chassis_left.get_position() + chassis_right.get_position() ); // in degrees

        // Signed distance error.
        error = target - curr;

        // Feed |error| to the exit condition and check it.
        //  - exit fires after |error| stays inside exitRange for exitTimeout ms.
        //  - earlyExitRange is an instantaneous "close enough" threshold,
        //    only meaningful when minSpeed > 0 (since with no floor the PID
        //    would keep pushing past it anyway).
        exit.update(std::fabs(error));

        if (exit.getExit())
            break;
        if (params.minSpeed > 0 && std::fabs(error) < params.earlyExitRange)
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
        move_voltage(output, output);

        // Print telemetry at most once every debugRefreshTime ms (0 = off).
        // out here is the post-clamp/floor voltage actually sent to the motors,
        // so during saturation it won't equal p_term + i_term + d_term — that
        // gap is your saturation signal.
        if (debugRefreshTime > 0 &&
            pros::millis() - lastDebug >= (uint32_t)debugRefreshTime)
        {
            debugPID("lat", error, output, pid);
            lastDebug = pros::millis();
        }

        pros::delay(10);
    }

    // Stop the motors. The actual stopping behavior (coast vs brake vs
    // hold) is whatever the caller set via Chassis::setBrakeMode.
    move_voltage(0, 0);
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    printf ("moveFor done, error = %.2f---\n", error);
}
