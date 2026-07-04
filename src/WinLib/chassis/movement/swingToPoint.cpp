#include "WinLib/chassis/chassis.hpp"
#include "WinLib/chassis/Odom.hpp"
#include "WinLib/pid.hpp"
#include "WinLib/exitcondition.hpp"
#include "WinLib/timer.hpp"
#include "WinLib/util.hpp"
#include "WinLib/debugPrint.hpp"
#include "WinLib/pose.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

using namespace WinLib;

/* =============================================================================
 * swingToPoint() — swing until the robot FACES an (x, y) point.
 *
 * swingToHeading, but the target heading is the BEARING to the point, recomputed
 * every tick (the robot translates as it swings, so the bearing changes). One
 * side is locked (brake-held) and only the other drives — the robot pivots
 * around the locked wheels.
 *
 *   lockedSide = the side held stationary. params.forwards = face the point with
 *   the front (true) or the back (false). Other AngularParams as in swingToHeading.
 *
 * COORDINATE CONVENTION (matches Odom.cpp): heading 0° = +Y, CW; the bearing to a
 * delta (dx, dy) is atan2(dx, dy) — x and y swapped vs the textbook atan2(y, x).
 *
 * Ported from LemLib's swingToPoint (see CHANGES_FROM_REFERENCE.md).
 * =========================================================================== */
void Chassis::swingToPoint(float x, float y, DriveSide lockedSide, int timeout, AngularParams params)
{
    ExitCondition exit(angularSettings.exitRange, angularSettings.exitTimeout);
    Timer timer(timeout);

    float error = 0;

    // Initial bearing to the point → gain-schedule kP ONCE (like turnToHeading).
    Pose  p0   = getPose();
    float aim0 = RadToDeg(std::atan2(x - p0.x, y - p0.y));
    if (!params.forwards) aim0 = std::fmod(aim0 + 180.0f, 360.0f);
    float initialError = angleError(aim0, getHeading(), /*radians=*/false, params.direction);
    float kP = angularSettings.gains
        ? asymptoticGain(std::fabs(initialError),
                         angularSettings.gains->initial,
                         angularSettings.gains->final,
                         angularSettings.gains->knee,
                         angularSettings.gains->power)
        : angularSettings.kP;

    angular_PID pid(kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange);

    // Brake mode HOLD up front so the locked (pivot) side actually holds.
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    if (debugRefreshTime > 0)
        printf("swingToPoint start: target=(%.0f, %.0f) lock=%s fwd=%d kP=%.4f\n",
               x, y, lockedSide == DriveSide::LEFT ? "L" : "R", params.forwards, kP);

    uint32_t lastDebug = 0;

    // Main control loop, 100 Hz (same cadence as odom).
    while (!timer.isDone())
    {
        // Recompute the bearing to the point from the CURRENT pose — the robot
        // moves while it swings, so the aim shifts.
        Pose  pose   = getPose();
        float target = RadToDeg(std::atan2(x - pose.x, y - pose.y));
        if (!params.forwards) target = std::fmod(target + 180.0f, 360.0f);  // wrap (see moveToPoint)

        error = angleError(target, getHeading(), /*radians=*/false, params.direction);

        exit.update(error);
        if (exit.getExit())
            break;
        if (std::fabs(error) < std::fabs(params.earlyExitRange))
            break;

        float output = pid.compute(error);
        output = clamp(output, params.maxSpeed, -params.maxSpeed);
        if (std::fabs(output) < std::fabs(params.minSpeed))
            output = params.minSpeed * sgn(output);

        // Swing mix: lock one side (brake-hold), drive only the other. Free-side
        // sign matches turnToHeading's rotation sense (left = +output, right = -output).
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
            printf("swingPt: tgtHead=%.1f\n", target);
            debugPID("ang", error, output, pid);
            printf("\n");
            lastDebug = pros::millis();
        }

        pros::delay(10);
    }

    move_voltage(0, 0);
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    printf("swingToPoint (%.0f, %.0f) done, error = %.2f, imu = %.2f---\n", x, y, error, getHeading());
    printf("batteryLevel: %.0f\n\n\n", pros::c::battery_get_capacity());
}
