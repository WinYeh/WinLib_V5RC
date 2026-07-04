#include "WinLib/chassis/chassis.hpp"
#include "WinLib/pid.hpp"
#include "WinLib/chassis/Odom.hpp"
#include "WinLib/exitcondition.hpp"
#include "WinLib/timer.hpp"
#include "WinLib/util.hpp"
#include "WinLib/debugPrint.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <algorithm>

using namespace WinLib;

/* =============================================================================
 * moveByWall() — drive a set distance while HUGGING a side wall.
 *
 * This is moveFor with the steering correction swapped out. moveFor holds an
 * IMU heading; moveByWall instead holds a fixed distance (`standoff`, mm) from a
 * side wall, read off a left/right distance sensor. The forward distance is
 * still measured by the drive ENCODERS (via MMTodeg), exactly like moveFor.
 *
 * Optional handoff: if `params.targetHead` is set, then once the robot is within
 * `alignThreshold` mm of the standoff, the correction switches from "match the
 * wall distance" to "hold this IMU heading" — useful for squaring up to a target
 * heading once you've settled against the wall.
 *
 * Ported from 14683A's VEXcode `move_new_wall()` (see CHANGES_FROM_REFERENCE.md).
 * Divergences: linear PID for the drive (not their |error|^e curve), a `side`
 * enum + `standoff` instead of their target_L/target_R-with-a-zero trick, and mm
 * throughout instead of cm.
 * =========================================================================== */
void Chassis::moveByWall(float distance, WallSide side, float standoff,
                         int timeout, WallParams params)
{
    /* ________________________________ INITIALIZATION _________________________________*/
    // Fresh exit condition + timer every call → clean integral / timer state.
    ExitCondition exit(lateralSettings.exitRange,
                       lateralSettings.exitTimeout);
    Timer timer(timeout);

    // Capture the starting encoder reading instead of taring (the drivetrain odom
    // mode reads these every tick — taring would corrupt its pose). Same trick as
    // moveFor: measure this move relative to the start offset.
    float lateral_start = 0.5 * (this->drivetrain.rightMotors->get_position()
                               + this->drivetrain.leftMotors->get_position()); // degrees

    /* ______________________________ LATERAL (distance) _________________________________*/
    float lateral_error  = 0;
    float lateral_target = MMTodeg(distance) * params.forwards; // motor degrees

    // Constant kP, or the gain-schedule curve if lateralSettings carries one.
    float lateral_kP = lateralSettings.gains
            ? asymptoticGain(std::fabs(lateral_target),
                             lateralSettings.gains->initial,
                             lateralSettings.gains->final,
                             lateralSettings.gains->knee,
                             lateralSettings.gains->power)
            : lateralSettings.kP;

    PID lateral_pid(lateral_kP, lateralSettings.kI,
                    lateralSettings.kD, lateralSettings.windupRange);

    // Leave headroom below 12 V so the wall correction (±1 V) never saturates the
    // drive on its own — same cap moveFor uses.
    params.maxSpeed = std::min(params.maxSpeed, 10.5f);

    /* ________________________________ WALL (correction) ________________________________*/
    // PD controller on the standoff error, in mm. kI = 0 (a slowly-integrated
    // wall offset isn't wanted — we just track the wall as we pass it).
    PID wall_pid(params.turnKp, 0, params.turnKd, 0);

    if (debugRefreshTime > 0)
        printf("moveByWall start: dist=%.0f side=%s standoff=%.0f kP=%.5f\n",
               distance, side == WallSide::LEFT ? "L" : "R", standoff, lateral_kP);

    uint32_t lastDebug = 0;

    // Main control loop, 100 Hz (same cadence as odom).
    while (!timer.isDone())
    {
        /* _____________________________________ LATERAL _______________________________________*/
        // Distance traveled since the move began (avg of both sides, minus the
        // start offset — the encoders are never tared).
        float lateral_curr = 0.5 * (this->drivetrain.rightMotors->get_position()
                                  + this->drivetrain.leftMotors->get_position()) - lateral_start;
        lateral_error = lateral_target - lateral_curr;

        exit.update(lateral_error);
        if (exit.getExit())
            break;
        if (std::fabs(lateral_error) < std::fabs(params.earlyExitRange))
            break;

        float lateral_output = lateral_pid.compute(lateral_error);
        lateral_output = clamp(lateral_output, params.maxSpeed, -params.maxSpeed);
        if (std::fabs(lateral_output) < std::fabs(params.minSpeed))
            lateral_output = params.minSpeed * sgn(lateral_output);

        /* ______________________________________ WALL _________________________________________*/
        // Read the chosen side wall through the DSR (it already owns the sensors).
        float d = (side == WallSide::LEFT) ? this->dsr.leftReading()
                                           : this->dsr.rightReading();

        float wallError   = 0;
        float turn_output = 0;
        bool  headingMode = false;

        if (d < 0)
        {
            // No sensor / invalid reading this tick: don't steer on garbage — just
            // drive straight (turn_output stays 0). Mirrors the odom glitch-guard idea.
        }
        else
        {
            // Standoff error, mm. Sign matches 14683A: LEFT → (standoff - d),
            // RIGHT → (d - standoff), so a positive error always steers the same
            // rotational way regardless of which wall we hug.
            wallError = (side == WallSide::LEFT) ? (standoff - d) : (d - standoff);

            // Once aligned, optionally hand off to IMU heading-hold.
            if (params.targetHead && std::fabs(wallError) < params.alignThreshold)
            {
                headingMode = true;
                float headErr = angleError(*params.targetHead, getHeading(),
                                           /*radians=*/false, AngularDirection::AUTO);
                turn_output = 0.05f * headErr;   // small proportional, like the reference
            }
            else
            {
                turn_output = wall_pid.compute(wallError);
            }
        }

        // Cap the correction so it trims the drive rather than overpowering it.
        turn_output = clamp(turn_output, +1.0, -1.0);

        /* __________________________________ INTEGRATION ____________________________________*/
        // Mix. Driving in reverse flips the correction sense (the same steering
        // command rotates the robot the other way relative to travel), so swap the
        // turn term for backward moves — matching 14683A's reverse branch.
        if (params.forwards >= 0)
            move_voltage(lateral_output + turn_output, lateral_output - turn_output);
        else
            move_voltage(lateral_output - turn_output, lateral_output + turn_output);

        /* __________________________________ DEBUG PRINT ____________________________________*/
        if (debugRefreshTime > 0 &&
            pros::millis() - lastDebug >= (uint32_t)debugRefreshTime)
        {
            printf("wall: side=%s d=%.0f err=%.1f turn=%.2f mode=%s\n",
                   side == WallSide::LEFT ? "L" : "R", d, wallError, turn_output,
                   headingMode ? "head" : "wall");
            debugPID("lat", lateral_error, lateral_output, lateral_pid);
            printf("\n");
            lastDebug = pros::millis();
        }

        pros::delay(10);
    }

    // Stop. Stopping behavior (coast/brake/hold) is whatever the caller set.
    move_voltage(0, 0);
    setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    printf("moveByWall done: lat_err=%.2f heading=%.2f\n", lateral_error, getHeading());
    printf("batteryLevel: %.0f\n\n\n", pros::c::battery_get_capacity());
}
