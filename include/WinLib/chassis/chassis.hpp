#pragma once

#include "pros/motors.hpp"                       // IWYU pragma: keep 
#include "pros/motor_group.hpp"                  // IWYU pragma: keep 
#include "pros/imu.hpp"                          // IWYU pragma: keep 
#include "WinLib/pose.hpp"                       // IWYU pragma: keep 
#include "WinLib/util.hpp"
#include "WinLib/pid.hpp"
#include "WinLib/exitcondition.hpp"
#include "WinLib/chassis/OdomSensors.hpp"        // IWYU pragma: keep
#include "WinLib/chassis/DSR.hpp"                // IWYU pragma: keep
#include <optional>

namespace WinLib {

// =========================================================================
// Config — Drivetrain + ControllerSettings
// =========================================================================

/**
 * @brief POD config for the physical drivetrain.
 *
 * Plain data, no methods. Constructed once at the robot-application layer
 * (e.g. in config.cpp) and handed to the Chassis constructor.
 *
 * @b Example
 * @code {.cpp}
 * pros::MotorGroup leftMotors  ({1, 2, 3});
 * pros::MotorGroup rightMotors ({4, 5, 6});
 *
 * WinLib::Drivetrain drivetrain {
 *     &leftMotors,
 *     &rightMotors,
 *     10,                          // track width (inches)
 *     WinLib::Omniwheel::NEW_325,  // 3.25" omnis
 *     360,                         // drivetrain rpm at the wheel
 *     2                            // horizontalDrift — 2 for all-omni, 8 with traction wheels
 * };
 * @endcode
 */
struct Drivetrain
{
    pros::MotorGroup* leftMotors;
    pros::MotorGroup* rightMotors;
    float trackWidth;       // inches — distance between left and right wheels
    float wheelDiameter;    // inches
    float rpm;              // rpm at the wheel (after gearing)
    float horizontalDrift;  // cornering grip ceiling, used by moveToPose (boomerang).
                            // 2 for all-omni, 8 with traction wheels.
};

/**
 * @brief Optional gain-schedule curve for a controller's kP (PIDPlus-style).
 *
 * Plain data, no methods. When a ControllerSettings carries one of these (its
 * `gains` optional has a value), the movement code computes kP ONCE at the start
 * of each motion from the motion's size, via WinLib::asymptoticGain() (see
 * pid.hpp for the curve math). When `gains` is empty, the controller just uses
 * its plain constant kP. So scheduling is purely opt-in per axis: give a
 * ControllerSettings one of these to enable it, omit it to keep the original
 * constant-kP behavior.
 *
 * Knee/setpoint units follow the motion's own error units — heading degrees for
 * turns, motor degrees for moveFor.
 */
struct AsymptoticGains {
    float initial;  // kP for small moves (near-zero error)
    float final;    // kP the curve approaches for large moves
    float knee;     // error size at the curve midpoint, (initial + final) / 2
    float power;    // transition sharpness (higher = sharper bend)
};

/**
 * @brief Settings for a single-axis controller (lateral or angular).
 *
 * Constructed from a `PID` and a single `ExitCondition` so the call site
 * reads as the actual mental model — "a controller is a PID plus one exit
 * window." Internally, the constructor pulls the numeric values out via
 * getters and stores them as flat floats. This means:
 *   - construction is self-documenting (no positional 6-float blob),
 *   - storage is plain data — every field is directly readable and writable
 *     by movement code (`settings.kP`, `settings.exitRange`, etc.),
 *   - a route can tune any field at runtime in one line:
 *       chassis.lateralSettings.kP = 15;
 *       chassis.lateralSettings.exitRange = 0.5;
 *
 * The `PID` and `ExitCondition` objects passed in are only used at
 * construction — they are NOT stored, and mutating them afterward has no
 * effect on the ControllerSettings. The movement code builds fresh `PID` and
 * `ExitCondition` instances from these flat fields each motion.
 *
 * @b Example
 * @code {.cpp}
 * WinLib::ControllerSettings lateralSettings(
 *     WinLib::PID(10, 0, 3, 3),       // kP, kI, kD, windupRange
 *     WinLib::ExitCondition(1, 100)   // settle: within 1 inch for 100 ms
 * );
 * @endcode
 */
class ControllerSettings {
public:
    // `gains` is optional: pass an AsymptoticGains to enable PIDPlus-style kP
    // scheduling, or omit it (the default) to use the constant `kP` pulled from
    // the PID. Stored by value, so it can be constructed inline at the call site.
    ControllerSettings(PID pid, ExitCondition exit,
                       std::optional<AsymptoticGains> gains = std::nullopt)
        : kP          (pid.getkP()),
          kI          (pid.getkI()),
          kD          (pid.getkD()),
          windupRange (pid.getWindupRange()),
          exitRange   (exit.getRange()),
          exitTimeout (exit.getTime()),
          gains       (gains) {}

    float kP;
    float kI;
    float kD;
    float windupRange;
    float exitRange;
    float exitTimeout;
    // empty (std::nullopt) -> use the constant kP above.
    // has a value          -> movement code picks kP from this curve per motion.
    std::optional<AsymptoticGains> gains;
};


// =========================================================================
// Motion parameter structs (named-argument idiom)
// =========================================================================
// Param structs let users override only the fields they care about, e.g.
//   chassis.moveToPoint(48, 24, 2000, {.forwards = false, .maxSpeed = 80});
// All fields have sensible defaults so a bare call like
//   chassis.moveToPoint(48, 24, 2000);
// also works.

struct LateralParams {
    /** Drive direction: 1 = forwards, -1 = backwards. */
    int   forwards       = 1;
    /** Cap on motor power, in volts. Hardware max is 12.0 V. */
    float maxSpeed       = 12.0;
    /** Floor on motor power once moving, in volts. Set non-zero to use smoother exit conditions. */
    float minSpeed       = 0;
    /** Exit early once within this distance of the target (only used if minSpeed > 0). */
    float earlyExitRange = 0;
};

struct AngularParams {
    /** Which way to turn. AUTO picks the shorter direction. */
    AngularDirection direction = AngularDirection::AUTO;
    /** For turnToPoint only: face the point with the front (true) or back (false). */
    bool  forwards       = true;
    /** Cap on motor power, in volts. Hardware max is 12.0 V. */
    float maxSpeed       = 12.0;
    /** Floor on motor power once turning, in volts. */
    float minSpeed       = 0;
    /** Exit early once within this angle of the target (only used if minSpeed > 0). */
    float earlyExitRange = 0;
};

struct MoveToPoseParams {
    /** Approach the pose facing forwards (true) or backwards (false). */
    bool  forwards       = true;
    /** Carrot-point multiplier, 0–1. Higher → curvier path. 0.6 is a sane default. */
    float lead           = 0.6;
    /** Cap on motor power, in volts. Hardware max is 12.0 V. */
    float maxSpeed       = 12.0;
    /** Floor on motor power, in volts. */
    float minSpeed       = 0;
    /** Exit early once within this distance of the target (only used if minSpeed > 0). */
    float earlyExitRange = 0;
};


// =========================================================================
// Chassis
// =========================================================================

/**
 * @brief Chassis class — the public API for autonomous motions and opcontrol drive.
 *
 * Owns the drivetrain hardware handles and the lateral/angular controller
 * settings. Reads pose by calling into the odom module's free functions
 * (`WinLib::getPose()` etc.); it does NOT own the pose itself.
 *
 * Typical setup (in initialize()):
 * @code {.cpp}
 * void initialize() {
 *     WinLib::init();         // start the odom tracking task
 *     chassis.calibrate();    // calibrate IMU
 * }
 * @endcode
 *
 * Typical use (in autonomous()):
 * @code {.cpp}
 * void autonomous() {
 *     chassis.moveToPoint(48, 24, 2000);
 *     chassis.turnToHeading(90, 1000, {.maxSpeed = 80});
 *     chassis.moveToPose(60, 60, 90, 4000, {.lead = 0.4});
 * }
 * @endcode
 */
class Chassis {
public:
    Chassis(Drivetrain drivetrain,
            OdomSensors odomSensors,
            ControllerSettings lateralSettings,
            ControllerSettings angularSettings,
            DSR dsr);

    // ---- setup / one-shots ----
    /** Calibrate the IMU. Call once in initialize(). */
    void calibrate(bool calibrateIMU = true);
    /** Set the drivetrain motors' brake mode. */
    void setBrakeMode(pros::motor_brake_mode_e mode);
    /** Reset the robot's (x, y) without touching heading. */
    void resetPosition();
    /** Motor cartridge free-speed rpm (100/200/600), read from the left drivetrain group. */
    float motorRPM();
    /** Convert millimeters to degrees based on the wheel diameter. */
    float MMTodeg(float distance);
    /** Convert motor-shaft degrees back to millimeters of travel (inverse of MMTodeg). */
    float degToMM(float degrees);   

    // ---- direct motor control (raw, bypasses PID and motion logic) ----
    /**
     * @brief Drive each side at a specific voltage.
     *
     * Parameter order is **right first, then left**.
     *
     * @param right voltage for the right side, in volts (typical ±12.0)
     * @param left  voltage for the left side,  in volts
     */
    void move_voltage(float left, float right);
    /**
     * @brief Drive each side at a percentage of max voltage (12 V).
     *
     * 100 = full forward (12 V), -100 = full reverse (-12 V), 0 = stop.
     * Parameter order is **right first, then left**.
     *
     * @param right percentage for the right side (-100 to +100)
     * @param left  percentage for the left side  (-100 to +100)
     */
    void move_percentage(float left, float right);

    // ---- autonomous motions (all blocking) ----
    void moveToPoint  (float x, float y,              int timeout, LateralParams   params = {});
    void moveFor      (float distance, float theta,   int timeout, LateralParams   params = {});
    void turnToHeading(float theta,                   int timeout, AngularParams   params = {});
    void turnBy       (float angle,                   int timeout, AngularParams   params = {});
    void turnToPoint  (float x, float y,              int timeout, AngularParams   params = {});
    void moveToPose   (float x, float y, float theta, int timeout, MoveToPoseParams params = {});

    // ---- opcontrol drive ----
    /**
     * @brief Single-method arcade drive — the simplest possible joystick control.
     *
     * Raw joystick input, no deadband, no drive curve, no scaling. Wires:
     *   left  motor = throttle + turn
     *   right motor = throttle - turn
     *
     * Call this once per opcontrol loop tick with the controller's analog
     * axis values (typically axis 3 = left-stick-Y for throttle and axis 1
     * = right-stick-X for turn).
     */
    void arcade(float throttle, float turn);

    // ---- runtime-tunable config (public on purpose) ----
    Drivetrain         drivetrain;
    OdomSensors        odomSensors;   // single source of truth — odom.cpp reads through chassis
    ControllerSettings lateralSettings;
    ControllerSettings angularSettings;
    DSR                dsr;
};

} // namespace WinLib
