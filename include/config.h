#pragma once
#include "WinLib/chassis/OdomSensors.hpp"
#include "api.h"                              // IWYU pragma: keep
#include "WinLib/chassis/chassis.hpp"         // IWYU pragma: keep

// ---- PROS devices ----
extern pros::Controller master;

extern pros::MotorGroup chassis_left, chassis_right;

extern pros::Imu imu1, imu2;
extern pros::Distance dist_F, dist_R, dist_L;
// TODO: add a back distance sensor (e.g. `dist_B`) to give DSR a 4th input.
// Until then, DSR only uses front/left/right.

// ---- WinLib chassis ----
// The Chassis bundles the Drivetrain, two ControllerSettings (lateral and
// angular), and the DSR. Constructed in config.cpp; used everywhere else.
extern WinLib::TrackingWheel vertical_tracking_wheel, horizontal_tracking_wheel; // TODO declare these in config.cpp and pass &them to odom_sensors once the tracking wheels are ready.
extern WinLib::OdomSensors odom_sensors;
extern WinLib::Drivetrain drivetrain;
extern WinLib::ControllerSettings lateralSettings, angularSettings;
extern WinLib::DSR dsr;
extern WinLib::Chassis chassis;
