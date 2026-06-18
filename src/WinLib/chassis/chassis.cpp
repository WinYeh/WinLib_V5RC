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
#include "pros/rtos.hpp"

using namespace WinLib;


/* ---- constructor ---- */
// Just copies the config into our member fields. The Drivetrain holds raw
// pointers to user-owned MotorGroups (and the DSR holds raw pointers to
// pros::Distance sensors), so the user must keep those alive for the
// lifetime of the Chassis (which is normally a file-scope global).
Chassis::Chassis(Drivetrain drivetrain,
                 OdomSensors odomSensors,
                 ControllerSettings lateralSettings,
                 ControllerSettings angularSettings,
                 DSR dsr)
    : drivetrain(drivetrain),
      odomSensors(odomSensors),
      lateralSettings(lateralSettings),
      angularSettings(angularSettings),
      dsr(dsr) {}


/* ---- calibrate ---- */
// Calibrate the IMU (blocking ~3 seconds), zero the tracking-wheel encoders,
// then spawn the odom tracking task. Each branch is null-guarded so a
// partially-wired robot (no horizontal wheel, etc.) still works.
//
// Note on order: IMU calibration must finish BEFORE the tracking task starts,
// otherwise the task's first ticks would read a non-calibrated IMU. Wheel
// resets can come either side of calibration — putting them before init() so
// the task's first read sees a clean (0, 0, 0) starting point.
void Chassis::calibrate(bool calibrateIMU) 
{
    if (calibrateIMU && odomSensors.imu != nullptr) 
    {
        odomSensors.imu->reset(true);   // true = blocking
    }
    if (odomSensors.vertical   != nullptr) odomSensors.vertical  ->reset();
    if (odomSensors.horizontal != nullptr) odomSensors.horizontal->reset();
    WinLib::OdomInit();  
    pros::delay(3000); 
}


/* ---- setBrakeMode ---- */
// Forwards to both motor groups. We guard against null in case a unit test
// or an in-progress wiring leaves a side unconnected.
void Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    if (drivetrain.leftMotors  != nullptr) drivetrain.leftMotors ->set_brake_mode(mode);
    if (drivetrain.rightMotors != nullptr) drivetrain.rightMotors->set_brake_mode(mode);
}


/* ---- resetPosition ---- */
// Delegates to the DSR module — it reads the 4 distance sensors and the
// current IMU heading, computes the robot's true (x, y) on the field, and
// pushes that back into the odom module via WinLib::setPose(). The robot's
// heading is left untouched (the IMU is the source of truth for heading).
//
// See WinLib/chassis/DSR.hpp for the algorithm and assumptions.
void Chassis::resetPosition() {
    dsr.reset();
}

/* This function converts millimeters to degrees based on the wheel diameter and 
only used for converting target distances to motor positions */
float Chassis::MMTodeg(float distance)
{
    float MOTOR_RPM = 0;
    switch(drivetrain.leftMotors->get_gearing() ) 
    {
        case pros::v5::MotorGears::red:
            MOTOR_RPM = 100;
            break;
        case pros::v5::MotorGears::green:
            MOTOR_RPM = 200;
            break;
        case pros::v5::MotorGears::blue:
            MOTOR_RPM = 600;
            break;
        default:
            MOTOR_RPM = 600; // default to 600 if gearing is unknown
    }
    float gearRatio = MOTOR_RPM / drivetrain.rpm;
    float wheelCircumferenceMM = drivetrain.wheelDiameter * M_PI * 25.4; // in mm
    return distance / wheelCircumferenceMM * 360.0f * gearRatio; // in degrees
}


/* ---- move_voltage ----
 * Drive each side at a specific voltage. Bypasses all PID and motion logic —
 * useful for testing, calibration, and writing custom motion routines that
 * need to set raw motor outputs directly.
 *
 * Note the parameter ORDER: left first, then right. The values are in volts
 * (typically -12.0 .. +12.0). PROS's `move_voltage` takes millivolts, so we
 * scale by 1000 here. PROS clips inputs outside ±12000 mV to those bounds
 * automatically, so no clamping is needed on our side.
 *
 * Null-guarded against partially-wired drivetrains.
 */
void Chassis::move_voltage(float left, float right) 
{
    if (drivetrain.leftMotors  != nullptr) 
        drivetrain.leftMotors ->move_voltage(left  * 1000);
    if (drivetrain.rightMotors != nullptr) 
        drivetrain.rightMotors->move_voltage(right * 1000);
}


/* ---- move_percentage ----
 * Drive each side at a percentage of max voltage. 100 = 12 V (full forward),
 * -100 = -12 V (full reverse), 0 = stop. Bypasses PID and motion logic.
 *
 * Same parameter order as move_voltage: left first, then right.
 *
 * Conversion: percentage × 12000 mV / 100 = percentage × 120 mV. PROS clips
 * overflow automatically.
 */
void Chassis::move_percentage(float left, float right) 
{
    if (drivetrain.leftMotors  != nullptr) 
        drivetrain.leftMotors ->move_voltage(left  * 120);
    if (drivetrain.rightMotors != nullptr) 
        drivetrain.rightMotors->move_voltage(right * 120);
}