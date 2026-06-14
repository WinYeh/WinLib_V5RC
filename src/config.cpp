#include "config.h"                    // IWYU pragma: keep
#include "WinLib/chassis/chassis.hpp"  // IWYU pragma: keep
#include "WinLib/chassis/DSR.hpp"      // IWYU pragma: keep
#include "WinLib/chassis/OdomSensors.hpp"  // IWYU pragma: keep
#include "WinLib/pid.hpp"              // IWYU pragma: keep
#include "WinLib/exitcondition.hpp"    // IWYU pragma: keep
#include "pros/abstract_motor.hpp"     // IWYU pragma: keep
#include "pros/distance.hpp"           // IWYU pragma: keep
#include "pros/motor_group.hpp"        // IWYU pragma: keep
#include "pros/motors.hpp"             // IWYU pragma: keep
#include "pros/optical.hpp"            // IWYU pragma: keep

/* PROS devices declaration and initialization.*/

// controller
pros::Controller master (pros::E_CONTROLLER_MASTER);

// chassis
pros::MotorGroup chassis_left ( {-1, 2, 12}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::MotorGroup chassis_right ( {4, -6, -13}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// imu
pros::Imu imu1 (5);
pros::Imu imu2 (11);

// distance sensors
pros::Distance dist_F (17);
pros::Distance dist_R (14);
pros::Distance dist_L (8);

/* End of PROS devices declaration and initialization.*/

/* WinLib chassis and sensors declaration and initialization. */

// ---- Drivetrain ----
// Holds raw pointers to the motor groups above. Track width and wheel
// dimensions describe the physical robot; horizontalDrift is the cornering
// grip ceiling used by boomerang.
// TODO: replace placeholder dimensions with measurements from the real robot.
WinLib::Drivetrain drivetrain {
    &chassis_left,
    &chassis_right,
    /*trackWidth=*/      11.0f,                       // inches — measure between left/right wheel centers
    /*wheelDiameter=*/   WinLib::Omniwheel::NEW_325,  // inches — swap if you're not on 3.25" omnis
    /*rpm=*/             450,                         // wheel rpm (green motor 200 × gear ratio)
    /*horizontalDrift=*/ 2                            // 2 for all-omni, 8 with traction wheels
};

// ---- Lateral controller (moveToPoint / moveFor / boomerang) ----
// Error in mm, output in volts (0–12). Gains are V per mm of error.
// TODO: tune kP/kI/kD on the real robot.
WinLib::ControllerSettings lateralSettings(
    WinLib::PID(
        /*kP=*/          0.005f,   // V/mm — TODO tune
        /*kI=*/          0,        // leave at 0 until you see steady-state drift
        /*kD=*/          0.0005f,  // V/(mm/tick) — TODO tune
        /*windupRange=*/ 50        // mm — only integrate when |error| < this
    ),
    WinLib::ExitCondition(
        /*range=*/ 10,             // mm — tight "we're done" window
        /*time=*/  100             // ms — dwell time before declaring done
    ),
    WinLib::ExitCondition(
        /*range=*/ 50,             // mm — wider "good enough" window
        /*time=*/  500             // ms — used when approach is slow
    )
);

// ---- Angular controller (turnToHeading / turnToPoint / boomerang) ----
// Error in degrees, output in volts. Gains are V per degree.
WinLib::ControllerSettings angularSettings(
    WinLib::PID(
        /*kP=*/          0.08f,    // V/deg — TODO tune
        /*kI=*/          0,
        /*kD=*/          0.008f,   // V/(deg/tick) — TODO tune
        /*windupRange=*/ 5         // deg
    ),
    WinLib::ExitCondition(
        /*range=*/ 1,              // deg — tight final-heading window
        /*time=*/  100             // ms
    ),
    WinLib::ExitCondition(
        /*range=*/ 5,              // deg — wider settle window
        /*time=*/  500             // ms
    )
);

// ---- DSR (Distance Sensor Reset) ----
// Currently 3 of 4 sides are instrumented (no back sensor). DSR.reset() skips
// any null sensor automatically, so this is safe; you just get fewer candidate
// readings per call until the back sensor is added.
// Each sensor takes (X, Y) offsets in robot frame: +y = forward, +x = the
// robot's LEFT side. See DSR.hpp for examples.
// TODO: measure each sensor's actual (X, Y) on the robot and replace below.
WinLib::DSR dsr(
    /*front=*/ &dist_F, /*frontOffsetX=*/ 0,    /*frontOffsetY=*/ 165,  // TODO measure
    /*back=*/  nullptr, /*backOffsetX=*/  0,    /*backOffsetY=*/  0,    // TODO: add a back sensor
    /*left=*/  &dist_L, /*leftOffsetX=*/  140,  /*leftOffsetY=*/  0,    // TODO measure
    /*right=*/ &dist_R, /*rightOffsetX=*/ -140, /*rightOffsetY=*/ 0     // TODO measure
);

// ---- OdomSensors ----
// Bundles the (future) tracking wheels and the IMU for the odom module.
// Tracking wheels aren't declared yet — passing nullptr is fine, the odom
// module will skip them. The IMU pointer is wired to imu1; imu2 stays
// declared but unused until we add dual-IMU averaging.
// IMPORTANT: declaring this object does NOT yet hand it to the odom module.
// odom.cpp still has its own internal `odomSensors` static set to nullptrs.
// The injection mechanism is deferred per CLAUDE.md § Deferred Decisions.
WinLib::OdomSensors odom_sensors(
    /*vertical=*/   nullptr,       // TODO: declare a TrackingWheel and pass &it
    /*horizontal=*/ nullptr,       // TODO: same
    /*imu=*/        &imu1
);

// ---- Chassis (wires it all together) ----
// Holds the Drivetrain, both ControllerSettings, and the DSR by value. The
// referenced devices (motor groups, distance sensors, IMU) are file-scope
// globals here, so they outlive the Chassis automatically.
WinLib::Chassis chassis(drivetrain, odom_sensors, lateralSettings, angularSettings, dsr);

/* WinLib chassis and sensors declaration and initialization. */
