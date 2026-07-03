#pragma once

#include "WinLib/pid.hpp"
#include "WinLib/pose.hpp"

namespace WinLib
{

/**
 * @brief global throttle for all debug printing, in milliseconds.
 *
 * A motion's debug block prints at most once every debugRefreshTime ms.
 * Set it to 0 (or less) to silence every debug line — this one knob both
 * throttles AND turns debugging off. Set it to e.g. 50 while tuning, 0 for a
 * real match. Defaults to 0 (off).
 */
extern int debugRefreshTime;

/**
 * @brief print one PID controller's telemetry for this tick to the terminal.
 *
 * Prints a single labeled line over USB (pros terminal):
 *   [label] t=..ms err=.. out=..V p_term=.. i_term=.. d_term=..
 * The P/I/D terms are read straight off the PID that just ran, so call this
 * right after pid.compute(). `err` is printed without a unit because its
 * meaning differs per motion (inches, motor-degrees, heading-degrees) — the
 * label tells you which controller it is. `out` is always volts.
 *
 * Stateless: it always prints when called. Throttling is the caller's job
 * (see debugRefreshTime and the throttle pattern in the motion files).
 *
 * @param label short tag for the controller, e.g. "lat" or "ang"
 * @param error the signed error fed to the PID this tick
 * @param output the PID output, in volts, this tick
 * @param pid the controller that just ran, read for its p/i/d term breakdown
 */
void debugPID(const char* label, float error, float output, const PID& pid);

/**
 * @brief print the robot's current pose vs the target pose to the terminal.
 *
 * For odom-based motions (turnToPoint, moveToPoint, moveToPose) where a single
 * error number doesn't show the whole picture. Prints:
 *   [pose] x=.. y=.. theta=..deg | tgt=(..,..) dist=..
 * x / y / dist are in whatever units odom reports (the same units you pass in);
 * theta is labeled as degrees, matching getPose()'s default.
 *
 * Stateless: it always prints when called; throttle at the call site.
 *
 * @param current the robot's current pose (e.g. from getPose())
 * @param target the pose being driven to
 */
void debugPose(const Pose& current, const Pose& target);

} // namespace WinLib
