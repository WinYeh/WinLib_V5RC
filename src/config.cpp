#include "config.h"                 // IWYU pragma: keep
#include "pros/abstract_motor.hpp"  // IWYU pragma: keep
#include "pros/distance.hpp"        // IWYU pragma: keep
#include "pros/motor_group.hpp"     // IWYU pragma: keep
#include "pros/motors.hpp"          // IWYU pragma: keep
#include "pros/optical.hpp"         // IWYU pragma: keep

// controller 
pros::Controller master (pros::E_CONTROLLER_MASTER);

// chassis 
pros::MotorGroup chassis_left ( {-1, 2, 12}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); 
pros::MotorGroup chassis_right ( {4, -6, -13}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); 

// imu 
pros::Imu imu1 (5);
pros::Imu imu2 (11);  

// distance sensors
pros::Distance dist_F (17);
pros::Distance dist_R (14);
pros::Distance dist_L (8);