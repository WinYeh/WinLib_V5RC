// chassis.cpp
//
// Glue methods of the Chassis class — constructor and the small one-shots
// that don't deserve their own file: calibrate, setBrakeMode, resetLocalPosition.
//
// The autonomous-motion methods (moveToPoint, turnToHeading, boomerang, ...)
// live in chassis/movement/<motion>.cpp.
// The opcontrol methods (tank, arcade, curvature) live in chassis/opcontrol.cpp.

#include "WinLib/chassis/chassis.hpp"
#include "WinLib/chassis/odom.hpp"

using namespace WinLib;


/* ---- constructor ---- */
// Just copies the config into our member fields. The Drivetrain holds raw
// pointers to user-owned MotorGroups, so the user must keep those alive for
// the lifetime of the Chassis (which is normally a file-scope global).
Chassis::Chassis(Drivetrain drivetrain,
                 ControllerSettings lateralSettings,
                 ControllerSettings angularSettings)
    : drivetrain(drivetrain),
      lateralSettings(lateralSettings),
      angularSettings(angularSettings) {}


/* ---- calibrate ---- */
// Starts the odom tracking task. IMU/tracking-wheel calibration is intentionally
// not wired up here yet — odomSensors lives inside the odom module as
// module-level static state, and the mechanism for injecting real sensors into
// it is being designed separately (see CLAUDE.md § Deferred Decisions: "Odom
// sensor injection"). Once that injection lands, this method should also call
// odomSensors.imu->reset(true) when calibrateIMU is true, plus reset the
// tracking wheels' encoder positions.
void Chassis::calibrate(bool calibrateIMU) {
    // (void) silences the "unused parameter" warning until the IMU
    // calibration is wired up.
    (void)calibrateIMU;
    WinLib::init();   // idempotent — spawns the tracking task once
}


/* ---- setBrakeMode ---- */
// Forwards to both motor groups. We guard against null in case a unit test
// or an in-progress wiring leaves a side unconnected.
void Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    if (drivetrain.leftMotors  != nullptr) drivetrain.leftMotors ->set_brake_mode(mode);
    if (drivetrain.rightMotors != nullptr) drivetrain.rightMotors->set_brake_mode(mode);
}


/* ---- resetLocalPosition ---- */
// Zero out (x, y) without touching theta. Useful for routes that want to
// re-anchor the field-relative origin partway through (e.g. after a wall
// alignment). We go through the odom module's public API rather than poking
// at its statics directly.
void Chassis::resetLocalPosition() {
    Pose p = WinLib::getPose();
    p.x = 0;
    p.y = 0;
    WinLib::setPose(p);
}
