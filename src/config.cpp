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

// testing chassis
pros::MotorGroup chassis_left ( {-2, -11, -12}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::MotorGroup chassis_right ( {10, 19, 20}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

// ace chassis
pros::MotorGroup chassis_left1 ( {-11, -12, -13}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::MotorGroup chassis_right1 ( {18, 19, 20}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

// dr4b chassis
pros::MotorGroup chassis_left2 ( {-11, -12, -13}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::MotorGroup chassis_right2 ( {14, 15, 16}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// imu — both wrapped in CustomIMU so the calibration scalar in
// CustomIMU::get_rotation() takes effect when odom reads through OdomSensors.
// imu1's scalar (1.01152008991) was carried over from Genesis 78181A's
// measured calibration. Re-measure on YOUR physical IMU before trusting it:
// spin the robot 10 full turns on a flat surface, read pros::c::imu_get_rotation,
// scalar = 3600 / measured.
// imu2 is placeholder (port 0, scalar 1.0) until dual-IMU averaging is wired up.
WinLib::CustomIMU imu1 (15, 1.0);   // scalar value genesis use = 1.01152008991, but you should re-measure on your physical IMU
WinLib::CustomIMU imu2 (0,  1.0);

// rotational sensors 
pros::Rotation rot_V (16);   // vertical encoder
pros::Rotation rot_H (14);   // horizontal encoder

// distance sensors
pros::Distance dist_F (0);
pros::Distance dist_R (0);
pros::Distance dist_L (0);

// DR4B Subsystems（Double Reverse 4 Bar）
pros::MotorGroup lifter ( {1, -2}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor wrist (10, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::adi::DigitalOut claw ('H', false); 

/* End of PROS devices declaration and initialization.*/

/* WinLib chassis and sensors declaration and initialization. */

// ---- Drivetrain ----
// Holds raw pointers to the motor groups above. Track width and wheel
// dimensions describe the physical robot; horizontalDrift is the cornering
// grip ceiling used by boomerang.
// TODO: replace placeholder dimensions with measurements from the real robot.
WinLib::Drivetrain dt {
    &chassis_left,
    &chassis_right,
    /*trackWidth=*/      11.0f,                       // inches — measure between left/right wheel centers
    /*wheelDiameter=*/   WinLib::Omniwheel::NEW_325,  // inches — swap if you're not on 3.25" omnis
    /*rpm=*/             360,                         // wheel rpm (green motor 200 × gear ratio)
    /*horizontalDrift=*/ 2                            // 2 for all-omni, 8 with traction wheels
};

// ---- Lateral controller (moveToPoint / moveFor / boomerang) ----
// Error in mm, output in volts (0–12). Gains are V per mm of error.
// The 3rd ctor arg is an optional AsymptoticGains — present here, so lateral
// motions schedule kP by move size (lateral scheduling ON). The constant kP
// below is the fallback boomerang uses (boomerang stays unscheduled for now).
//
// NOTE: the knee is in the motion's OWN error units. moveFor works in MOTOR
// degrees (via MMTodeg, ~1 motor-deg per mm on 3.25" wheels), so knee is in
// motor degrees, NOT mm — re-derive if you add an mm-based lateral motion.
// TODO: tune everything on the real robot.
WinLib::ControllerSettings lateralSett(
    WinLib::linear_PID(
        /*kP=*/          0.027f,   // V/mm — fallback when scheduling off — TODO tune
        /*kI=*/          0,        // leave at 0 until you see steady-state drift
        /*kD=*/          0.0000f,  // V/(mm/tick) — TODO tune
        /*windupRange=*/ 50        // mm — only integrate when |error| < this
    ),
    WinLib::ExitCondition(
        /*range=*/ 10,             // degrees — settle window
        /*time=*/  100             // ms — dwell time before declaring done
    )//,
    // WinLib::AsymptoticGains{
    //     /*initial=*/ 0.006f,       // snappy kP for short moves
    //     /*final=*/   0.028f,       // gentle kP for long moves 
    //     /*knee=*/    350.0f,       // MOTOR deg of error at curve midpoint
    //     /*power=*/   1.5f          // transition sharpness
    // }
);

// ---- Angular controller (turnToHeading / turnToPoint / boomerang) ----
// Error in degrees, output in volts. Gains are V per degree.
// The 3rd ctor arg enables kP scheduling on turns (the axis that needed it):
// big swings get a low kP (gentle), tiny corrections get a high kP (snappy).
// The constant kP below is the fallback boomerang uses.
WinLib::ControllerSettings angularSett(
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
        /*final=*/   0.165f,        // gentle kP for big swings (≈ 0.17 found by hand) — TODO tune
        /*knee=*/    90.0f,        // deg of error at curve midpoint — TODO tune
        /*power=*/   5.f          // transition sharpness — TODO tune
    }
);

// ---- OdomSensors ----
// Bundles the tracking wheels and the IMU for the odom module.
// Tracking wheels aren't declared yet — passing nullptr is fine, the odom
// module will skip them. The IMU pointer is wired to imu1; imu2 stays
// declared but unused until we add dual-IMU averaging.
WinLib::TrackingWheel vertical_tracking_wheel(
    &rot_V,                         // encoder
    WinLib::Omniwheel::NEW_2,      // diameter (inches)
    0                                // offset from center (mm) — TODO find out a way to measure the center of rottation of a robot
);   
WinLib::TrackingWheel horizontal_tracking_wheel( 
    &rot_H,                         // encoder
    WinLib::Omniwheel::NEW_2,      // diameter (inches)
    0                                // offset from center (mm) - TODO find out a way to measure the center of rottion of a robot
);   
WinLib::OdomSensors odom_sensors(
    /*vertical=*/   &vertical_tracking_wheel,       // TODO: declare a TrackingWheel and pass &it
    /*horizontal=*/ &horizontal_tracking_wheel,       // TODO: same
    /*imu=*/        &imu1
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

// ---- Chassis (wires it all together) ----
// Holds the Drivetrain, both ControllerSettings, and the DSR by value. The
// referenced devices (motor groups, distance sensors, IMU) are file-scope
// globals here, so they outlive the Chassis automatically.
WinLib::Chassis chassis(
            dt, 
           odom_sensors, 
       lateralSett, 
       angularSett, 
               /*dsr=*/ dsr
);

/* End of WinLib chassis and sensors declaration and initialization. */
