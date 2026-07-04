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
 * moveToPose() — drive to (x, y) AND arrive facing heading `theta`.
 * (This is the "boomerang" controller; LemLib exposes it under this name.)
 *
 * THE ONE TRICK: the robot never chases the real target. It chases a fake,
 * moving target called the CARROT, placed a little BEHIND the real target along
 * the final-approach line. Steering toward the carrot naturally bends the path
 * so the robot lines up with `theta` by the time it arrives.
 *
 *        ↑ theta (final facing)
 *        |
 *   target ●
 *       🥕  ← carrot: behind the target, along the approach line
 *       🤖  ← robot chases the carrot, not the target
 *
 * The carrot's distance behind the target is (lead * distanceToTarget). As the
 * robot closes in, distanceToTarget → 0, so the carrot slides FORWARD and merges
 * into the real target. That collapsing carrot is what makes the path a clean
 * arc that ends exactly on the dot.
 *
 * THIS IS THE MINIMAL TEACHING VERSION. It intentionally leaves out two things
 * the reference libraries (LemLib / Genesis) do — see CHANGES_FROM_REFERENCE.md:
 *   1. the sgn(cos) "commit to the curve at full speed while far" trick, and
 *   2. the horizontalDrift curvature-based slip-speed clamp.
 * It keeps: the carrot, collapse-on-close, and the swap to the final heading
 * near the end. Constant kP (no gain scheduling) per the project's decision.
 *
 * COORDINATE CONVENTION (important — matches Odom.cpp): heading is a COMPASS
 * angle — 0° points along +Y, and it increases CLOCKWISE. So the forward unit
 * vector for a heading H is (sin H, cos H), not the textbook (cos H, sin H).
 * Every sin/cos below follows that convention.
 * =========================================================================== */
void Chassis::moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params)
{
    /* ________________________________ INITIALIZATION _________________________________*/
    // Fresh exit condition + timer every call → clean integral / timer state.
    // We reuse the LATERAL settings' exit window; the exit metric is the
    // distance still left to the target (converted to motor degrees, so the
    // same exitRange you tuned for moveFor means the same thing here).
    ExitCondition exit(lateralSettings.exitRange,
                       lateralSettings.exitTimeout);
    Timer timer(timeout);

    // The target pose. theta is wrapped into [0, 360) just for tidy debug prints;
    // angleError() handles wrapping on its own regardless.
    Pose target(x, y, std::fmod(theta, 360.0f) < 0
                          ? std::fmod(theta, 360.0f) + 360.0f
                          : std::fmod(theta, 360.0f));

    // Constant kP on both axes — no asymptotic gain schedule. This matches
    // Genesis's movePosePlus, which hardcodes setKp (setKp(1) lateral, setKp(180)
    // turn) so its asymptotic "curve" just evaluates to a fixed constant too.
    // Nothing stops us from scheduling the lateral kP off the initial straight-line
    // distance later; we keep it constant to match the reference and stay simple.
    PID         lateral_pid(lateralSettings.kP, lateralSettings.kI,
                            lateralSettings.kD, lateralSettings.windupRange);
    angular_PID angular_pid(angularSettings.kP, angularSettings.kI,
                            angularSettings.kD, angularSettings.windupRange);

    // `close` is true whenever the robot is within this radius of the target
    // (recomputed each tick). It drives the endgame: the carrot collapses onto
    // the target and steering switches from "face the carrot" to "face the final
    // theta". 190 mm ≈ 7.5 in, the same radius LemLib uses. (With the default
    // minSpeed = 0 the PID decelerates into the radius and stays there, so not
    // latching this is fine; if you add a large minSpeed and see the carrot
    // flicker back open on an overshoot, latch it instead.)
    const float closeRadius = 190.0f; // mm
    bool  close          = false;

    float lateralError   = 0; // mm left to the target (for debug / exit)
    float angularError   = 0; // degrees

    if (debugRefreshTime > 0)
        printf("boomerang start: target=(%.0f, %.0f) theta=%.1f lead=%.2f\n",
               target.x, target.y, target.theta, params.lead);

    uint32_t lastDebug = 0;

    // Main control loop, 100 Hz (same cadence as odom).
    while (!timer.isDone())
    {
        /* _____________________________________ POSE ________________________________________*/
        Pose  pose       = getPose();               // x, y in mm; theta in degrees
        float distTarget = pose.distance(target);   // mm straight-line to target

        // Are we inside the settle radius this tick?
        close = distTarget < closeRadius;

        /* ____________________________________ CARROT _______________________________________*/
        // While far: place the carrot (lead * distTarget) behind the target along
        // the final heading. While close: carrot == target (it has collapsed).
        // Forward vector for compass heading is (sin, cos) — see file header.
        Pose carrot = target;
        if (!close)
        {
            float theta = DegToRad(target.theta);
            carrot.x = target.x - std::sin(theta) * params.lead * distTarget;
            carrot.y = target.y - std::cos(theta) * params.lead * distTarget;
        }

        /* ____________________________________ ANGULAR ______________________________________*/
        // Which part of the robot we aim: its front (forwards) or, for a
        // backwards approach, its back (heading + 180°).
        // Keep this a valid [0, 360) heading: getHeading() + 180 can reach ~540°,
        // and angleError only wraps once, so it can't fix an error that's off by
        // more than a full turn. fmod folds the back-heading back into range.
        float robotHeading = params.forwards ? getHeading()
                                             : std::fmod(getHeading() + 180.0f, 360.0f);

        // What we point that part AT:
        //   far  → the carrot (this is what bends the path)
        //   close→ the final theta (stop chasing the carrot; lock the ending
        //          heading, and avoid the atan2 jitter you'd get aiming at a
        //          carrot that's almost on top of you).
        float aimHeading;
        if (close)
        {
            aimHeading = target.theta;
        }
        else
        {
            // Compass heading from robot to carrot. With the (sin,cos) forward
            // convention, the heading toward a delta (dx,dy) is atan2(dx, dy) —
            // x and y swapped vs the usual atan2(y, x).
            float dx = carrot.x - pose.x;
            float dy = carrot.y - pose.y;
            aimHeading = RadToDeg(std::atan2(dx, dy));
        }

        angularError = angleError(aimHeading, robotHeading, /*radians=*/false,
                                  AngularDirection::AUTO);

        /* ____________________________________ LATERAL ______________________________________*/
        // How far to drive this tick: the straight-line distance to the target,
        // scaled by cos(angularError). Facing the target → cos≈1 → full push.
        // Facing 90° off → cos≈0 → don't drive (turn first). This is the simple,
        // always-on cosine scaling (we skip the reference's far-away sgn trick).
        lateralError = distTarget * std::cos(DegToRad(angularError));
        // Backwards approach: the same distance, driven in reverse.
        if (!params.forwards) lateralError = -lateralError;

        /* __________________________________ EXIT CHECK _____________________________________*/
        // Exit metric is the remaining distance, in motor degrees, so it reuses
        // the lateral exitRange. We only allow exit once `close` — otherwise a
        // fast fly-by near the target could satisfy the window too early.
        float exitMetric = MMTodeg(distTarget);
        exit.update(exitMetric);
        if (exit.getExit() && close)
            break;
        // earlyExitRange is an instantaneous "close enough" distance in mm,
        // only meaningful when minSpeed > 0 (with no floor the PID coasts in).
        if (distTarget < std::fabs(params.earlyExitRange))
            break;

        /* _________________________________ PID → VOLTS _____________________________________*/
        // Lateral PID is tuned in motor-degree space (like moveFor), so convert
        // the mm error before feeding it in.
        float lateral_output = lateral_pid.compute(MMTodeg(lateralError));
        lateral_output = clamp(lateral_output, params.maxSpeed, -params.maxSpeed);
        if (std::fabs(lateral_output) < std::fabs(params.minSpeed))
            lateral_output = params.minSpeed * sgn(lateral_output);

        float angular_output = angular_pid.compute(angularError);

        /* __________________________________ INTEGRATION ____________________________________*/
        // Classic differential mix, then desaturate: if the combined command
        // would exceed maxSpeed on either side, scale BOTH sides down by the
        // same ratio so the turn shape is preserved.
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
            printf("boom: dist=%.0f close=%d carrot=(%.0f,%.0f)",
                   distTarget, close, carrot.x, carrot.y);
            debugPID("lat", MMTodeg(lateralError), lateral_output, lateral_pid);
            debugPID("ang", angularError,          angular_output, angular_pid);
            printf("\n");
            lastDebug = pros::millis();
        }

        pros::delay(10);
    }

    // Stop. Final stopping behavior (coast/brake/hold) is whatever the caller
    // set via Chassis::setBrakeMode.
    move_voltage(0, 0);
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    printf("boomerang done: dist=%.2f heading=%.2f (target theta=%.1f)\n",
           lateralError, getHeading(), target.theta);
    printf("batteryLevel: %.0f\n\n\n", pros::c::battery_get_capacity());
}
