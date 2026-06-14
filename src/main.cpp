#include "main.h"
#include "config.h"                    // IWYU pragma: keep

void initialize() 
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Welcome to WinLib (v0.0.1).");
}

void disabled() 
{
	
}

void competition_initialize() 
{

}

void autonomous() 
{

}

void opcontrol() 
{

	while (true)
	{
		float throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   // axis 3
		float turn     = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.arcade(throttle, turn);
		pros::delay(10);                               // Run for 20 ms then update
	}
}