#include "WinLib/chassis/chassis.hpp"
#include "WinLib/pid.hpp"
#include "WinLib/chassis/Odom.hpp"
#include "WinLib/exitcondition.hpp"
#include "WinLib/timer.hpp"
#include "WinLib/util.hpp"
#include "WinLib/debugPrint.hpp"
#include "pros/rtos.hpp"
#include <cmath>

using namespace WinLib;

void Chassis::moveFor(float distance, float theta, int timeout, LateralParams params)
{
    /* ________________________________ INITIALIZATION _________________________________*/
    // Build the exit condition + timer from the chassis's lateral settings.
    // Fresh objects every call → clean integral / timer state.
    ExitCondition exit(lateralSettings.exitRange,
                       lateralSettings.exitTimeout);

    Timer timer(timeout);

    // Capture the starting encoder reading instead of taring.
    // The drivetrain encoders are a SHARED resource: the drivetrain odom mode
    // reads them every tick to derive heading from the left/right difference.
    // Taring (resetting the counter to 0) would look to that reader like the
    // robot suddenly lurched backward, corrupting the pose. So we leave the
    // counters running continuously and measure this move's distance relative
    // to the start offset captured here.
    float lateral_start = 0.5 * (this->drivetrain.rightMotors->get_position()
                               + this->drivetrain.leftMotors->get_position()); // in degrees

    /* ______________________________ LATERAL INITIALIZATION _____________________________*/
    float lateral_error = 0;
    // Store converted distance in mm to motor's degree as target for the PID controller
    float lateral_target = MMTodeg(distance) * params.forwards; // in degrees

    // The commanded travel (|target|, in motor degrees) is the "size" of this
    // move. If lateral settings carry a gain-schedule curve, pick kP from that
    // size ONCE here; otherwise fall back to the plain constant kP. Either way
    // kP is fixed for the whole motion.
    float lateral_kP = lateralSettings.gains
            ? asymptoticGain(std::fabs(lateral_target),
                             lateralSettings.gains->initial,
                             lateralSettings.gains->final,
                             lateralSettings.gains->knee,
                             lateralSettings.gains->power)
            : lateralSettings.kP;

    PID lateral_pid(lateral_kP,
                    lateralSettings.kI,
                    lateralSettings.kD,
                    lateralSettings.windupRange);
    
    /* _______________________________ ANGULAR INITIALIZATION _________________________________*/

    float angular_error = 0;
    // Pre-wrap the target into [0, 360°) so a caller passing -30 or 720 still
    // produces the same intended heading. fmod can return negative, so add
    // 360 if it does.
    float angular_target = std::fmod(theta, 360.0f);
    if (angular_target < 0) angular_target += 360.0f;

    // The initial heading error is the "size" of this turn. If angular settings
    // carry a gain-schedule curve, pick kP from that size ONCE here (a big swing
    // gets a gentle kP, a tiny correction a snappy one); otherwise fall back to
    // the plain constant kP. Either way kP is fixed for the whole motion.
    float initial_angularError = angleError(angular_target, getHeading(), /*radians=*/false, AngularDirection::AUTO);
    float angular_kP = angularSettings.gains
                  ? asymptoticGain(std::fabs(initial_angularError),
                                    angularSettings.gains->initial,
                                      angularSettings.gains->final,
                                       angularSettings.gains->knee,
                                      angularSettings.gains->power)
                  : angularSettings.kP;

    angular_PID angular_pid(angular_kP,
                            angularSettings.kI,
                            angularSettings.kD,
                            angularSettings.windupRange);
    
    // One-time scheduling readout (only when debug is on).
    if (debugRefreshTime > 0)
        printf("moveFor start: target=%.2f kP=%.5f\n", lateral_target, lateral_kP);

    // Debug throttle clock (T1): the motion owns the timing, so the whole
    // mechanism is visible. lastDebug holds the time of the last print.
    uint32_t lastDebug = 0;

    // Main control loop, 100 Hz (same cadence as odom).
    while (!timer.isDone())
    {
        /* _____________________________________ LATERAL _______________________________________*/
        // Distance traveled since the move began. Average the two sides of the
        // chassis, then subtract the start offset (the encoders are never tared,
        // so the raw reading carries the whole match's accumulated rotation).
        float lateral_curr = 0.5 * (this->drivetrain.rightMotors->get_position() + this->drivetrain.leftMotors->get_position() ) - lateral_start; // in degrees

        // Signed distance error.
        lateral_error = lateral_target - lateral_curr;

        // Feed |error| to the exit condition and check it.
        //  - exit fires after |error| stays inside exitRange for exitTimeout ms.
        //  - earlyExitRange is an instantaneous "close enough" threshold,
        //    only meaningful when minSpeed > 0 (since with no floor the PID
        //    would keep pushing past it anyway).
        exit.update(lateral_error);

        if (exit.getExit())
            break;
        if (std::fabs(lateral_error) < std::fabs(params.earlyExitRange))
            break;

        // PID output, in volts.
        float lateral_output = lateral_pid.compute(lateral_error);

        // Clamp to [-maxSpeed, +maxSpeed]. maxSpeed is in volts; the
        // hardware ceiling is 12 V, but the caller may want a slower turn.
        lateral_output = clamp(lateral_output, params.maxSpeed, -params.maxSpeed);

        // Apply the min-speed floor.
        if (std::fabs(lateral_output) < std::fabs(params.minSpeed) )
        {
            lateral_output = params.minSpeed * sgn(lateral_output);
        }

        /* __________________________________ ANGULAR ________________________________________ */

        // Current heading, already wrapped into [0, 360°) by getHeading().
        float angular_curr = getHeading();

        // Signed angular error.
        // AUTO → shortest-path value in [-180°, +180°].
        // CW_CLOCKWISE / CCW_COUNTERCLOCKWISE → forced-direction error.
        angular_error = angleError(angular_target, angular_curr, /*radians=*/false, AngularDirection::AUTO);

        // PID output, in volts.
        float angular_output = angular_pid.compute(angular_error);        

        // Clamp to [-maxSpeed, +maxSpeed]. maxSpeed is in volts; the
        // hardware ceiling is 12 V, but the caller may want a slower turn.
        angular_output = clamp(angular_output, +2.0, -2.0);     
        
        /* __________________________________ INTEGRATION ____________________________________ */

        // Mix for in-place rotation (see sign convention at top of file).
        // Chassis::move_voltage signature is (left, right).
        move_voltage(lateral_output + angular_output, lateral_output - angular_output);

        /* __________________________________ DEBUG PRINT ____________________________________ */

        // Print telemetry at most once every debugRefreshTime ms (0 = off).
        // out here is the post-clamp/floor voltage actually sent to the motors,
        // so during saturation it won't equal p_term + i_term + d_term — that
        // gap is your saturation signal.
        if (debugRefreshTime > 0 &&
            pros::millis() - lastDebug >= (uint32_t)debugRefreshTime)
        {
            debugPID("lat", lateral_error, lateral_output, lateral_pid);
            debugPID("ang", angular_error, angular_output, angular_pid);
            lastDebug = pros::millis();
        }

        pros::delay(10);
    }
    // Stop the motors. The actual stopping behavior (coast vs brake vs
    // hold) is whatever the caller set via Chassis::setBrakeMode.
    move_voltage(0, 0);
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    printf ("moveFor done, error = %.2f, heading = %.2f\n", lateral_error, getHeading());
    printf ("batteryLevel: %.0f\n", pros::c::battery_get_capacity());
}
