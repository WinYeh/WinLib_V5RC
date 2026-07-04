#include "WinLib/chassis/chassis.hpp"
#include "WinLib/chassis/Odom.hpp"
#include "WinLib/pid.hpp"
#include "WinLib/exitcondition.hpp"
#include "WinLib/timer.hpp"
#include "WinLib/util.hpp"
#include "WinLib/debugPrint.hpp"
#include "WinLib/pose.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <algorithm>

using namespace WinLib;

/* =============================================================================
 * moveToPoint() — drive to (x, y). No target heading.
 *
 * The simpler sibling of moveToPose. It just aims the robot AT the point and
 * drives there; whatever heading it ends up facing is fine. LemLib splits these
 * into two motions for exactly this reason:
 *   - moveToPoint(x, y)        → "get to this spot" (final heading doesn't matter)
 *   - moveToPose (x, y, theta) → "get to this spot AND face this way" (carrot)
 * So moveToPoint is moveToPose minus the carrot, the lead, and the final-heading
 * lock — just "point at the target and drive."
 *
 * HOW IT WORKS each tick:
 *   1. angularError = angle from the robot's leading end to the point.
 *   2. lateralError = distanceToPoint * cos(angularError). The cos does 3 jobs:
 *        - facing the point (0°)     → cos ≈ 1  → full drive
 *        - 90° off                   → cos ≈ 0  → basically turn in place first
 *        - point slips BEHIND you    → cos < 0  → drive REVERSES, which pulls the
 *                                       robot back onto an overshoot with no
 *                                       special-case code.
 *   3. Feed both errors to their PIDs, mix (left = lat+ang, right = lat-ang),
 *      then desaturate to maxSpeed.
 *
 * CONSTANT kP (no gain scheduling), matching moveToPose / Genesis — see moveToPose.cpp.
 *
 * COORDINATE CONVENTION (matches Odom.cpp): heading is a COMPASS angle — 0° is
 * +Y, increasing CLOCKWISE, so the forward unit vector for a heading H is
 * (sin H, cos H) and the heading toward a delta (dx, dy) is atan2(dx, dy) — x and
 * y swapped vs the textbook atan2(y, x).
 * =========================================================================== */
void Chassis::moveToPoint(float x, float y, int timeout, LateralParams params)
{
    /* ________________________________ INITIALIZATION _________________________________*/
    // Fresh exit condition + timer every call → clean integral / timer state.
    // Exit metric is the remaining distance (in motor degrees), so it reuses the
    // LATERAL settings' exit window — the same exitRange you tuned for moveFor.
    ExitCondition exit(lateralSettings.exitRange, lateralSettings.exitTimeout);
    Timer timer(timeout);

    Pose target(x, y, 0);   // the heading field is unused — this is a POINT, not a pose

    // Constant kP, like moveToPose (see that file for the Genesis-matching reason).
    PID         lateral_pid(lateralSettings.kP, lateralSettings.kI,
                            lateralSettings.kD, lateralSettings.windupRange);
    angular_PID angular_pid(angularSettings.kP, angularSettings.kI,
                            angularSettings.kD, angularSettings.windupRange);

    // LateralParams::forwards is ±1: >= 0 → arrive front-first, < 0 → back-first.
    bool forwards = params.forwards >= 0;

    // Inside this radius we stop STEERING (but keep driving). Aiming at a point you
    // are almost on top of makes atan2 jitter wildly, and there is no final heading
    // to hold, so steering here would only spin the robot. The cos-scaled lateral
    // below still coasts us in and still reverses on an overshoot. Tune if the
    // robot wobbles at the end (raise it) or drifts off-point on approach (lower it).
    // 150 mm ≈ 0.5 ft — the distance Genesis's movePointPlus stops its angular term.
    const float closeRadius = 150.0f; // mm (≈ 0.5 ft, matching Genesis movePointPlus)
    bool  close        = false;

    float lateralError = 0; // mm remaining, cos-scaled (for the drive + debug)
    float angularError = 0; // degrees

    if (debugRefreshTime > 0)
        printf("moveToPoint start: target=(%.0f, %.0f) forwards=%d\n", x, y, forwards);

    uint32_t lastDebug = 0;

    // Main control loop, 100 Hz (same cadence as odom).
    while (!timer.isDone())
    {
        /* _____________________________________ POSE ________________________________________*/
        Pose  pose       = getPose();               // x, y in mm; theta in degrees
        float distTarget = pose.distance(target);   // mm straight-line to the point
        close = distTarget < closeRadius;

        /* ____________________________________ ANGULAR ______________________________________*/
        // Aim the leading end (front, or the back if reversing) straight at the
        // point — no carrot, that is moveToPose's job.
        // Keep this a valid [0, 360) heading: getHeading() + 180 can reach ~540°,
        // and angleError only wraps once, so it can't fix an error that's off by
        // more than a full turn. fmod folds the back-heading back into range.
        float robotHeading = forwards ? getHeading()
                                      : std::fmod(getHeading() + 180.0f, 360.0f);
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float aimHeading = RadToDeg(std::atan2(dx, dy));   // (sin,cos) convention
        angularError = angleError(aimHeading, robotHeading, /*radians=*/false,
                                  AngularDirection::AUTO);

        /* ____________________________________ LATERAL ______________________________________*/
        // Distance scaled by cos(angularError) (see file header). Keep computing
        // angularError even when close — the cos SIGN is what reverses the drive on
        // an overshoot; only the angular OUTPUT gets zeroed below.
        lateralError = distTarget * std::cos(DegToRad(angularError));
        if (!forwards) lateralError = -lateralError;

        /* __________________________________ EXIT CHECK _____________________________________*/
        // Only allow exit once `close`, so a fast fly-by near the point can't
        // satisfy the settle window too early.
        float exitMetric = MMTodeg(distTarget);
        exit.update(exitMetric);
        if (exit.getExit() && close)
            break;
        // earlyExitRange is an instantaneous "close enough" distance in mm,
        // only meaningful when minSpeed > 0 (with no floor the PID coasts in).
        if (distTarget < std::fabs(params.earlyExitRange))
            break;

        /* _________________________________ PID → VOLTS _____________________________________*/
        // Lateral PID is tuned in motor-degree space (like moveFor), so convert the
        // mm error before feeding it in.
        float lateral_output = lateral_pid.compute(MMTodeg(lateralError));
        lateral_output = clamp(lateral_output, params.maxSpeed, -params.maxSpeed);
        if (std::fabs(lateral_output) < std::fabs(params.minSpeed))
            lateral_output = params.minSpeed * sgn(lateral_output);

        float angular_output = angular_pid.compute(angularError);
        // Stop steering once basically on the point (see closeRadius note). Zero
        // ONLY the angular output — the cos-scaled lateral above still finishes.
        if (close) angular_output = 0;

        /* __________________________________ INTEGRATION ____________________________________*/
        // Classic differential mix, then desaturate: if the combined command would
        // exceed maxSpeed on either side, scale BOTH sides down by the same ratio
        // so the turn shape is preserved.
        float leftPower  = lateral_output + angular_output;
        float rightPower = lateral_output - angular_output;
        float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) { leftPower /= ratio; rightPower /= ratio; }

        // Chassis::move_voltage signature is (left, right).
        move_voltage(leftPower, rightPower);

        /* __________________________________ DEBUG PRINT ____________________________________*/
        if (debugRefreshTime > 0 &&
            pros::millis() - lastDebug >= (uint32_t)debugRefreshTime)
        {
            debugPose(pose, target);   // [pose] x=.. y=.. theta=..deg | tgt=(..,..) dist=..
            printf("mtp: dist=%.0f close=%d\n", distTarget, close);
            debugPID("lat", MMTodeg(lateralError), lateral_output, lateral_pid);
            debugPID("ang", angularError,          angular_output, angular_pid);
            printf("\n"); 
            lastDebug = pros::millis();
        }

        pros::delay(10);
    }

    // Stop. Final stopping behavior (coast/brake/hold) is whatever the caller set
    // via Chassis::setBrakeMode.
    move_voltage(0, 0);
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    printf("moveToPoint done: dist=%.2f heading=%.2f\n", lateralError, getHeading());
    printf("batteryLevel: %.0f\n\n\n", pros::c::battery_get_capacity());
}
