#include "main.h"
#include "config.h"                    // IWYU pragma: keep
#include "subsystems/dr4b.hpp"         // IWYU pragma: keep

void initialize() 
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Welcome to WinLib (v0.0.1).");
	dr4b::init();
	printf("project initilized---\n");
}

void disabled() 
{

}

void competition_initialize() 
{ 
    chassis.calibrate();
	printf ("chassis calibrated---\n");
}

void autonomous() 
{
	printf ("auton begins---\n"); 
	chassis.turnToHeading(90, 2000, {.maxSpeed = 12.0, .minSpeed = 0.5, .earlyExitRange = 2}); 
	pros::delay(1000);
	chassis.turnToHeading(180, 2000, {.maxSpeed = 12.0, .minSpeed = 0.5, .earlyExitRange = 2}); 
	pros::delay(1000);	 
	chassis.turnToHeading(0, 2000, {.maxSpeed = 12.0, .minSpeed = 0.5, .earlyExitRange = 2}); 
}

void opcontrol() 
{
	competition_initialize(); // for testing purposes
	autonomous(); 			  // for testing purposes
	while (true)
	{
		float throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		float turn     = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.arcade(throttle, turn);

		dr4b::Wrist::Ctr();
		dr4b::Claw::Ctr();	
		dr4b::Lifter::Ctr();
	}
}