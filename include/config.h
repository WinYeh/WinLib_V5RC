#pragma once 
#include "ALL.h"                // IWYU pragma: keep

extern pros::Controller master;

extern pros::MotorGroup chassis_left, chassis_right; 

extern pros::Motor roller, upper, back;     

extern pros::adi::Pneumatics Hood, Wing, Loader, IntakeLift;

extern pros::Imu imu1, imu2;
extern pros::Distance dist_F, dist_R, dist_L;
extern pros::Optical optic_Loader;