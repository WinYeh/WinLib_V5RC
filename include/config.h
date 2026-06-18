#pragma once
#include "WinLib/chassis/OdomSensors.hpp"
#include "api.h"                              // IWYU pragma: keep
#include "WinLib/chassis/chassis.hpp"         // IWYU pragma: keep
#include "pros/motor_group.hpp"

// ---- PROS devices ----
extern pros::Controller master;

extern pros::MotorGroup chassis_left, chassis_right;    // testing chassis 
extern pros::MotorGroup chassis_left1, chassis_right1;  // ace chassis
extern pros::MotorGroup chassis_left2, chassis_right2;  // dr4b chassis

extern WinLib::CustomIMU imu1, imu2;
extern pros::Rotation rot_V, rot_H;
extern pros::Distance dist_F, dist_R, dist_L;

// DR4B Subsystems（Double Reverse 4 Bar）
extern pros::MotorGroup lifter;
extern pros::Motor wrist;
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
