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

/* =============================================================================
 * swingToHeading() — turn to an absolute heading by LOCKING one side.
 *
 * Identical control to turnToHeading (same angular PID on angleError), but the
 * output drives only ONE side of the drivetrain while the other is brake-held.
 * The robot pivots around the locked wheels, so it changes heading AND carves
 * forward/backward at the same time — a swing turn, not an in-place spin.
 *
 *   lockedSide = the side held stationary (DriveSide::LEFT or RIGHT). The OTHER
 *                side is driven. params (AngularParams) reuse turnToHeading's
 *                (direction / maxSpeed / minSpeed / earlyExitRange). `forwards`
 *                is ignored here (it only matters for swingToPoint).
 *
 * Ported from LemLib's swingToHeading (see CHANGES_FROM_REFERENCE.md).
 * =========================================================================== */
void Chassis::swingToHeading(float theta, DriveSide lockedSide, int timeout, AngularParams params)
{
    ExitCondition exit(angularSettings.exitRange, angularSettings.exitTimeout);
    Timer timer(timeout);

    float error = 0;
    // Pre-wrap the target into [0, 360°) (same as turnToHeading).
    float target = std::fmod(theta, 360.0f);
    if (target < 0) target += 360.0f;

    // Gain-schedule kP off the initial turn size ONCE, or use the constant kP.
    float initialError = angleError(target, getHeading(), /*radians=*/false, params.direction);
    float kP = angularSettings.gains
        ? asymptoticGain(std::fabs(initialError),
                         angularSettings.gains->initial,
                         angularSettings.gains->final,
                         angularSettings.gains->knee,
                         angularSettings.gains->power)
        : angularSettings.kP;

    angular_PID pid(kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange);

    // Brake mode HOLD up front so brake()-ing the locked side actually holds it
    // (the pivot point), instead of letting it coast.
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    if (debugRefreshTime > 0)
        printf("swingToHeading start: target=%.1f lock=%s initErr=%.2f kP=%.4f\n",
               target, lockedSide == DriveSide::LEFT ? "L" : "R", initialError, kP);

    uint32_t lastDebug = 0;

    // Main control loop, 100 Hz (same cadence as odom).
    while (!timer.isDone())
    {
        float curr = getHeading();
        error = angleError(target, curr, /*radians=*/false, params.direction);

        exit.update(error);
        if (exit.getExit())
            break;
        if (std::fabs(error) < std::fabs(params.earlyExitRange))
            break;

        float output = pid.compute(error);
        output = clamp_Signed(output, params.maxSpeed, params.minSpeed);

        // Swing mix: lock one side (brake-hold), drive only the other. The free-
        // side sign matches turnToHeading's rotation sense (left = +output,
        // right = -output), so `direction`/angleError behave exactly as in a turn.
        if (lockedSide == DriveSide::LEFT)
        {
            if (drivetrain.leftMotors)  drivetrain.leftMotors->brake();
            if (drivetrain.rightMotors) drivetrain.rightMotors->move_voltage(-output * 1000);
        }
        else
        {
            if (drivetrain.rightMotors) drivetrain.rightMotors->brake();
            if (drivetrain.leftMotors)  drivetrain.leftMotors->move_voltage(+output * 1000);
        }

        if (debugRefreshTime > 0 &&
            pros::millis() - lastDebug >= (uint32_t)debugRefreshTime)
        {
            debugPID("ang", error, output, pid);
            printf("\n");
            lastDebug = pros::millis();
        }

        pros::delay(10);
    }

    move_voltage(0, 0);
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    printf("swingToHeading (%.1f) done, error = %.2f, imu = %.2f---\n", target, error, getHeading());
    printf("batteryLevel: %.0f\n\n\n", pros::c::battery_get_capacity());
}
