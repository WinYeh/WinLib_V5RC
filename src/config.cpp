#include "config.h"                 // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

// controller 
pros::Controller master (pros::E_CONTROLLER_MASTER);

// chassis 
pros::MotorGroup chassis_left ( {-1, 2, 12}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); 
pros::MotorGroup chassis_right ( {4, -6, -13}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); 

// intake 
pros::Motor roller (15, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); 
pros::Motor upper (-7, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); 
pros::Motor back (6, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees); 

// pneumatics
pros::adi::Pneumatics Hood ('a', false); 
pros::adi::Pneumatics Wing ('b', false); 
pros::adi::Pneumatics Loader ('c', false); 
pros::adi::Pneumatics IntakeLift ('d', false); 

// imu 
pros::Imu imu1 (5);
pros::Imu imu2 (11);  

// distance sensors
pros::Distance dist_F (17);
pros::Distance dist_R (14);
pros::Distance dist_L (8);

// optical sensors 
pros::Optical optic_Loader (10); 