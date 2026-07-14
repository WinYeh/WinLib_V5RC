#pragma once
#include "WinLib/chassis/OdomSensors.hpp"
#include "api.h"                              // IWYU pragma: keep
#include "WinLib/chassis/chassis.hpp"         // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

// ---- PROS devices ----
extern pros::Controller master;


// ---- Active chassis selection ----
// WinLib's robot-agnostic code (Odom.cpp, moveFor.cpp, ...) can only talk to ONE
// chassis, but config.h declares three (test::chassis, dr4b::chassis,
// ace::chassis). `Chs` is a single pointer that aims at whichever robot is in
// use. Call setActiveChassis(...) ONCE in initialize() (in main.cpp) to pick it,
// then the library reads through `Chs->...`.
//
// NOTE: this pointer only abstracts the CHASSIS (every namespace's `chassis` is
// the same WinLib::Chassis type, so one pointer can swap between them). It does
// NOT abstract subsystems — dr4b and ace have different subsystem types/APIs, so
// subsystem code still names its robot explicitly (e.g. ace::Cascade::Ctr()).
extern WinLib::Chassis* Chs;
void setActiveChassis(WinLib::Chassis& c);


namespace test
{
    extern pros::MotorGroup chassis_left, chassis_right; 

    extern WinLib::CustomIMU imu1, imu2;
    extern pros::Rotation rot_V, rot_H;
    extern pros::Distance dist_F, dist_R, dist_L;

    // ---- WinLib chassis ----
    // The Chassis bundles the Drivetrain, two ControllerSettings (lateral and
    // angular), and the DSR. Constructed in config.cpp; used everywhere else.
    extern WinLib::TrackingWheel vertical_tracking_wheel, horizontal_tracking_wheel; 
    extern WinLib::OdomSensors odom_sensors;
    extern WinLib::Drivetrain drivetrain;
    extern WinLib::ControllerSettings lateralSettings, angularSettings;
    extern WinLib::DSR dsr;
    extern WinLib::Chassis chassis;
}

namespace dr4b
{
    extern pros::MotorGroup chassis_left, chassis_right;

    extern WinLib::CustomIMU imu1, imu2;
    extern pros::Rotation rot_V, rot_H;
    extern pros::Distance dist_F, dist_R, dist_L;

    // DR4B subsystems
    extern pros::Motor           intake; 
    extern pros::MotorGroup      lifter;
    extern pros::Motor           wrist;
    extern pros::adi::DigitalOut claw;

    // ---- WinLib chassis ----
    // The Chassis bundles the Drivetrain, two ControllerSettings (lateral and
    // angular), and the DSR. Constructed in config.cpp; used everywhere else.
    extern WinLib::TrackingWheel vertical_tracking_wheel, horizontal_tracking_wheel; 
    extern WinLib::OdomSensors odom_sensors;
    extern WinLib::Drivetrain drivetrain;
    extern WinLib::ControllerSettings lateralSettings, angularSettings;
    extern WinLib::DSR dsr;
    extern WinLib::Chassis chassis;
}

namespace ace
{
    extern pros::MotorGroup chassis_left, chassis_right;

    extern WinLib::CustomIMU imu1, imu2;
    extern pros::Rotation rot_V, rot_H;
    extern pros::Distance dist_F, dist_R, dist_B, dist_L;

    // ace subsystems
    extern pros::Motor           intake; 
    extern pros::MotorGroup      ladybrown; 
    extern pros::Motor           cascade;
    extern pros::adi::DigitalOut claw;     
    
    // ---- WinLib chassis ----
    // The Chassis bundles the Drivetrain, two ControllerSettings (lateral and
    // angular), and the DSR. Constructed in config.cpp; used everywhere else.
    extern WinLib::TrackingWheel vertical_tracking_wheel, horizontal_tracking_wheel; 
    extern WinLib::OdomSensors odom_sensors;
    extern WinLib::Drivetrain drivetrain;
    extern WinLib::ControllerSettings lateralSettings, angularSettings;
    extern WinLib::DSR dsr;
    extern WinLib::Chassis chassis;
}


namespace catherine 
{
    extern pros::MotorGroup chassis_left, chassis_right; 

    extern WinLib::CustomIMU imu1, imu2;
    extern pros::Rotation rot_V, rot_H;
    extern pros::Distance dist_F, dist_R, dist_L;

    // catherine subsystems
    extern pros::Motor           intake; 
    extern pros::Motor           cascade;
    extern pros::adi::DigitalOut claw;     
    extern pros::Motor           wrist; 

    // ---- WinLib chassis ----
    // The Chassis bundles the Drivetrain, two ControllerSettings (lateral and
    // angular), and the DSR. Constructed in config.cpp; used everywhere else.
    extern WinLib::OdomSensors odom_sensors;
    extern WinLib::Drivetrain drivetrain;
    extern WinLib::ControllerSettings lateralSettings, angularSettings;
    extern WinLib::DSR dsr;
    extern WinLib::Chassis chassis;
}