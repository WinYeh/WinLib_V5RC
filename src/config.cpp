#include "config.h"                        // IWYU pragma: keep
#include "WinLib/chassis/chassis.hpp"      // IWYU pragma: keep
#include "WinLib/chassis/DSR.hpp"          // IWYU pragma: keep
#include "WinLib/chassis/OdomSensors.hpp"  // IWYU pragma: keep
#include "WinLib/pid.hpp"                  // IWYU pragma: keep
#include "WinLib/exitcondition.hpp"        // IWYU pragma: keep
#include "pros/abstract_motor.hpp"         // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/distance.hpp"               // IWYU pragma: keep
#include "pros/motor_group.hpp"            // IWYU pragma: keep
#include "pros/motors.hpp"                 // IWYU pragma: keep
#include "pros/optical.hpp"                // IWYU pragma: keep

/* PROS devices declaration and initialization.

   Everything is grouped by robot inside its own namespace (test / dr4b / ace),
   matching config.h. Each robot carries a complete, independent set of devices
   and its own WinLib::Chassis. Because the choice of active robot happens at
   runtime (via `Chs`), ALL of these objects are constructed at boot — that is
   harmless for PROS, whose Motor/Rotation/Distance objects are just lightweight
   handles. The controller is the only shared, robot-agnostic device. */

// controller (shared across all robots)
pros::Controller master (pros::E_CONTROLLER_MASTER);


// ---- Active chassis pointer ----
// Starts null. main.cpp's initialize() calls setActiveChassis(...) to aim it at
// one of the three chassis below, BEFORE anything (odom task, motions) reads it.
WinLib::Chassis* Chs = nullptr;
void setActiveChassis(WinLib::Chassis& c) { Chs = &c; }   // & = "address of"


/* ============================================================================
   test robot — a bare chassis with no subsystems, used for movement/odom work.
   ============================================================================ */
namespace test
{
    // chassis
    pros::MotorGroup chassis_left  ( {-2, -11, -12}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
    pros::MotorGroup chassis_right ( {9, 19, 20},    pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

    // imu — both wrapped in CustomIMU so the calibration scalar takes effect when
    // odom reads through OdomSensors. Re-measure on YOUR physical IMU before
    // trusting the scalar: spin the robot 10 full turns on a flat surface, read
    // pros::c::imu_get_rotation, scalar = 3600 / measured.
    // imu2 is a placeholder (port 0, scalar 1.0) until dual-IMU averaging is wired up.
    WinLib::CustomIMU imu1 (15, 1.0);   // genesis measured ≈ 1.01152008991 — re-measure yours
    WinLib::CustomIMU imu2 (0,  1.0);

    // rotational sensors
    pros::Rotation rot_V (16);   // vertical encoder
    pros::Rotation rot_H (14);   // horizontal encoder

    // distance sensors
    pros::Distance dist_F (0);
    pros::Distance dist_R (0);
    pros::Distance dist_L (0);

    // ---- Drivetrain ----
    // Holds raw pointers to the motor groups above. Track width and wheel
    // dimensions describe the physical robot; horizontalDrift is the cornering
    // grip ceiling used by moveToPose.
    // TODO: replace placeholder dimensions with measurements from the real robot.
    WinLib::Drivetrain drivetrain {
        &chassis_left,
        &chassis_right,
        /*trackWidth=*/      12.0f,                       // inches — measure between left/right wheel centers
        /*wheelDiameter=*/   WinLib::Omniwheel::NEW_325,  // inches
        /*rpm=*/             360,                         // wheel rpm (motor rpm × gear ratio)
        /*horizontalDrift=*/ 2                            // 2 for all-omni, 8 with traction wheels
    };

    // ---- Lateral controller (moveToPoint / moveFor / moveToPose) ----
    // Error in mm, output in volts (0–12). Gains are V per mm of error.
    // NOTE: the knee is in the motion's OWN error units. moveFor works in MOTOR
    // degrees (via MMTodeg, ~1 motor-deg per mm on 3.25" wheels), so knee is in
    // motor degrees, NOT mm. TODO: tune everything on the real robot.
    WinLib::ControllerSettings lateralSettings(
        WinLib::linear_PID(
            /*kP=*/          0.027f,   // V/mm — fallback when scheduling off
            /*kI=*/          0,        // leave at 0 until you see steady-state drift
            /*kD=*/          0.0000f,  // V/(mm/tick)
            /*windupRange=*/ 50        // mm — only integrate when |error| < this (positive only)
        ),
        WinLib::ExitCondition(
            /*range=*/ 10,             // degrees — settle window
            /*time=*/  100             // ms — dwell time before declaring done
        )
    );

    // ---- Angular controller (turnToHeading / turnToPoint / moveToPose) ----
    // Error in degrees, output in volts. Gains are V per degree. The 3rd ctor arg
    // enables kP scheduling on turns: big swings get a low kP (gentle), tiny
    // corrections get a high kP (snappy). The constant kP is the fallback moveToPose uses.
    WinLib::ControllerSettings angularSettings(
        WinLib::angular_PID(
            /*kP=*/          0.21f,    // fallback when scheduling off
            /*kI=*/          0,
            /*kD=*/          1.2f,
            /*windupRange=*/ 5         // deg
        ),
        WinLib::ExitCondition(
            /*range=*/ 1.5,            // deg — settle window
            /*time=*/  100             // ms — dwell time before declaring done
        ),
        WinLib::AsymptoticGains{
            /*initial=*/ 0.26f,        // snappy kP for tiny corrections — TODO tune
            /*final=*/   0.165f,       // gentle kP for big swings — TODO tune
            /*knee=*/    90.0f,        // deg of error at curve midpoint — TODO tune
            /*power=*/   5.f           // transition sharpness — TODO tune
        }
    );

    // ---- OdomSensors ----
    // Bundles the tracking wheels and the IMU for the odom module. The IMU
    // pointer is wired to imu1; imu2 stays declared but unused until we add
    // dual-IMU averaging.
    WinLib::TrackingWheel vertical_tracking_wheel(
        &rot_V,                    // encoder
        WinLib::Omniwheel::NEW_2, // diameter (inches)
        1.6                         // sideways offset from center (mm) — from the spin test (≈ x-residual / 2); near-centered, minor
    );
    WinLib::TrackingWheel horizontal_tracking_wheel(
        &rot_H,                    // encoder
        WinLib::Omniwheel::NEW_2,  // diameter (inches)
        -47                         // fwd/back offset from center (mm) — from the 180° spin test (≈ −Δy / 2)
    );
    // mode = VPD (single vertical wheel + drivetrain). Switch to WinLib::OdomMode::TW2
    // for the two-tracking-wheel algorithm (handy for comparing during validation).
    WinLib::OdomSensors odom_sensors(
        /*vertical=*/   &vertical_tracking_wheel,
        /*horizontal=*/ &horizontal_tracking_wheel,
        /*imu=*/        &imu1,
        /*mode=*/       WinLib::OdomMode::VPD,
        /*imuTrust=*/   0.98f      // IMU weight in the heading blend (0~1)
    );

    // ---- DSR (Distance Sensor Reset) ----
    // 3 of 4 sides instrumented (no back sensor). DSR.reset() skips any null
    // sensor. Each sensor takes (X, Y) offsets in robot frame: +y = forward,
    // +x = the robot's LEFT side. TODO: measure each sensor's actual (X, Y).
    WinLib::DSR dsr(
        /*front=*/ &dist_F, /*frontOffsetX=*/ 0,    /*frontOffsetY=*/ 165,
        /*back=*/  nullptr, /*backOffsetX=*/  0,    /*backOffsetY=*/  0,
        /*left=*/  &dist_L, /*leftOffsetX=*/  140,  /*leftOffsetY=*/  0,
        /*right=*/ &dist_R, /*rightOffsetX=*/ -140, /*rightOffsetY=*/ 0
    );

    // ---- Chassis (wires it all together) ----
    WinLib::Chassis chassis(
        drivetrain,
        odom_sensors,
        lateralSettings,
        angularSettings,
        /*dsr=*/ dsr
    );
}


/* ============================================================================
   dr4b robot — Double Reverse 4 Bar (lifter + wrist + claw + intake).
   ============================================================================ */
namespace dr4b
{
    // chassis (*note: trackwidth: 11.45in)
    pros::MotorGroup chassis_left  ( {-11, -12, -13}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    pros::MotorGroup chassis_right ( {14, 15, 16},    pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

    // imu (see note in test::imu1 about re-measuring the scalar)
    WinLib::CustomIMU imu1 (15, 1.0);
    WinLib::CustomIMU imu2 (0,  1.0);

    // rotational sensors
    pros::Rotation rot_V (16);   // vertical encoder
    pros::Rotation rot_H (14);   // horizontal encoder

    // distance sensors
    pros::Distance dist_F (0);
    pros::Distance dist_R (0);
    pros::Distance dist_L (0);

    // DR4B subsystems (Double Reverse 4 Bar)
    pros::Motor           intake (0, pros::v5::MotorGears::blue);
    pros::MotorGroup      lifter ( {1, -2}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    pros::Motor           wrist (10, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    pros::adi::DigitalOut claw ('H', false);

    // ---- Drivetrain ----  TODO: measure real dimensions (trackwidth ≈ 11.45in)
    WinLib::Drivetrain drivetrain {
        &chassis_left,
        &chassis_right,
        /*trackWidth=*/      11.45f,                      // inches
        /*wheelDiameter=*/   WinLib::Omniwheel::NEW_325,  // inches
        /*rpm=*/             200,                         // green motor wheel rpm — TODO set from gear ratio
        /*horizontalDrift=*/ 2
    };

    // ---- Lateral / angular controllers ----  TODO: tune on the real robot.
    WinLib::ControllerSettings lateralSettings(
        WinLib::linear_PID(/*kP=*/ 0.027f, /*kI=*/ 0, /*kD=*/ 0.0000f, /*windupRange=*/ 50),
        WinLib::ExitCondition(/*range=*/ 10, /*time=*/ 100)
    );
    WinLib::ControllerSettings angularSettings(
        WinLib::angular_PID(/*kP=*/ 0.21f, /*kI=*/ 0, /*kD=*/ 1.2f, /*windupRange=*/ 5),
        WinLib::ExitCondition(/*range=*/ 1.5, /*time=*/ 100),
        WinLib::AsymptoticGains{ /*initial=*/ 0.26f, /*final=*/ 0.165f, /*knee=*/ 90.0f, /*power=*/ 5.f }
    );

    // ---- OdomSensors ----
    WinLib::TrackingWheel vertical_tracking_wheel(   &rot_V, WinLib::Omniwheel::NEW_2, 0);
    WinLib::TrackingWheel horizontal_tracking_wheel( &rot_H, WinLib::Omniwheel::NEW_2, 47);
    WinLib::OdomSensors odom_sensors(
        /*vertical=*/ &vertical_tracking_wheel, /*horizontal=*/ &horizontal_tracking_wheel,
        /*imu=*/ &imu1, /*mode=*/ WinLib::OdomMode::TW2, /*imuTrust=*/ 0.98f
    );

    // ---- DSR ----  TODO: measure each sensor's (X, Y) offset.
    WinLib::DSR dsr(
        /*front=*/ &dist_F, /*frontOffsetX=*/ 0,    /*frontOffsetY=*/ 165,
        /*back=*/  nullptr, /*backOffsetX=*/  0,    /*backOffsetY=*/  0,
        /*left=*/  &dist_L, /*leftOffsetX=*/  140,  /*leftOffsetY=*/  0,
        /*right=*/ &dist_R, /*rightOffsetX=*/ -140, /*rightOffsetY=*/ 0
    );

    // ---- Chassis ----
    WinLib::Chassis chassis(drivetrain, odom_sensors, lateralSettings, angularSettings, /*dsr=*/ dsr);
}


/* ============================================================================
   ace robot — ladybrown + cascade + claw + intake.
   ============================================================================ */
namespace ace
{
    // chassis
    pros::MotorGroup chassis_left  ( {-10, -9, -8}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
    pros::MotorGroup chassis_right ( {2, 3, 4},     pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

    // imu (see note in test::imu1 about re-measuring the scalar)
    WinLib::CustomIMU imu1 (7, 1.0);
    WinLib::CustomIMU imu2 (0,  1.0);

    // rotational sensors
    pros::Rotation rot_V (6);   // vertical encoder
    pros::Rotation rot_H (0);   // horizontal encoder

    // distance sensors
    pros::Distance dist_F (0);
    pros::Distance dist_B (0); 
    pros::Distance dist_R (0);
    pros::Distance dist_L (0);

    // ace subsystems
    pros::Motor           intake ({}, pros::v5::MotorGears::blue);
    pros::MotorGroup      ladybrown ( {-19, 11}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    pros::Motor           cascade (20, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    pros::adi::DigitalOut claw ('A', false);

    // ---- Drivetrain ----  
    WinLib::Drivetrain drivetrain {
        &chassis_left,
        &chassis_right,
        /*trackWidth=*/      11.45f,                      // inches
        /*wheelDiameter=*/   WinLib::Omniwheel::NEW_275,  // inches
        /*rpm=*/             450,                         // wheel rpm
        /*horizontalDrift=*/ 2
    };

    // ---- Lateral / angular controllers ----  TODO: tune on the real robot.
    WinLib::ControllerSettings lateralSettings(
        WinLib::linear_PID(/*kP=*/ 0.01f, /*kI=*/ 0, /*kD=*/ 0.05f, /*windupRange=*/ 50),
        WinLib::ExitCondition(/*range=*/ 10, /*time=*/ 100)
        // WinLib::AsymptoticGains{ /*initial=*/ 0.2f, /*final=*/ 0.09f, /*knee=*/ 90.0f, /*power=*/ 5.f }
    );
    WinLib::ControllerSettings angularSettings(
        WinLib::angular_PID(/*kP=*/ 0.15f, /*kI=*/ 0.0f, /*kD=*/ 0.425f, /*windupRange=*/ 5),
        WinLib::ExitCondition(/*range=*/ 1.5, /*time=*/ 100),
        WinLib::AsymptoticGains{ /*initial(30deg)=*/ 0.2f, /*final(180deg)=*/ 0.09f, /*knee=*/ 90.0f, /*power=*/ 5.f }
    );

    // ---- OdomSensors ----
    WinLib::TrackingWheel vertical_tracking_wheel(&rot_V, WinLib::Omniwheel::NEW_2, 0);
    WinLib::OdomSensors odom_sensors(
        /*vertical=*/ &vertical_tracking_wheel, /*horizontal=*/ nullptr,
        /*imu=*/ &imu1, /*mode=*/ WinLib::OdomMode::VPD, /*imuTrust=*/ 0.98f
    );

    // ---- DSR ----  TODO: measure each sensor's (X, Y) offset.
    WinLib::DSR dsr(
        /*front=*/ &dist_F, /*frontOffsetX=*/ 0,    /*frontOffsetY=*/ 0,
        /*back=*/  &dist_B, /*backOffsetX=*/  0,    /*backOffsetY=*/  0,
        /*left=*/  &dist_L, /*leftOffsetX=*/  0,    /*leftOffsetY=*/  0,
        /*right=*/ &dist_R, /*rightOffsetX=*/ 0,    /*rightOffsetY=*/ 0
    );

    // ---- Chassis ----
    WinLib::Chassis chassis(drivetrain, odom_sensors, lateralSettings, angularSettings, /*dsr=*/ dsr);
}

/* End of PROS devices + WinLib chassis declaration and initialization. */
