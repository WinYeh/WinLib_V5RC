#include "main.h"
#include "WinLib/chassis/Odom.hpp"
#include "config.h"                    // IWYU pragma: keep
#include "pros/abstract_motor.hpp"	   // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "subsystems/dr4b.hpp"         // IWYU pragma: keep
#include "subsystems/ace.hpp"          // IWYU pragma: keep

void initialize()
{
	// Pick which robot is active BEFORE anything reads the chassis. The odom
	// task doesn't start until competition_initialize()'s calibrate(), which
	// runs after this, so Chs is always set in time.
	setActiveChassis(test::chassis);

	pros::lcd::initialize();
	pros::lcd::set_text(1, "Welcome to WinLib (v0.0.1).");
	printf("project initilized---\n");
}

void disabled() 
{

}

void competition_initialize() 
{
	test::rot_V.set_reversed(true);   // vertical wheel is wired reversed; flip it so driving forward reads + 
    Chs->calibrate(true);
	WinLib::setPose(WinLib::Pose(0, 0, 0));   // ensure odom starts with a known pose (x=0, y=0, heading=0)
	printf ("chassis calibrated---\n");
}

void autonomous() 
{
	printf ("auton begins---\n");
	Chs->turnToHeading(180, 2000, {.maxSpeed = 12.0, .minSpeed = 0.5, .earlyExitRange = 2});
	pros::delay(1000);
	/*
	Chs->moveFor(1500, 0, 5000, {.forwards = 1, .maxSpeed = 12.0, .minSpeed = 2.0, .earlyExitRange = 5});
	Chs->moveFor(1500, 0, 5000, {.forwards = -1, .maxSpeed = 12.0, .minSpeed = 2.0, .earlyExitRange = 5});
	*/

	WinLib::Pose pose = WinLib::getPose(false);
	printf("getPose() = (%.1f, %.1f, %.1f)\n", pose.x, pose.y, pose.theta);
}

void opcontrol() 
{
	competition_initialize(); // for testing purposes
	ace::cascade.tare_position(); 
	float last_update = pros::millis();

	while (true)
	{	
		/*if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) 
		{
			Chs->turnBy(180, 2000, {.maxSpeed = 12.0, .minSpeed = 0.5, .earlyExitRange = 2});
			pros::delay(1000);	
			WinLib::Pose pose = WinLib::getPose(false); 
			printf("getPose() = (%.1f, %.1f, %.1f)\n", pose.x, pose.y, pose.theta);
		}*/

		float throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		float turn     = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		Chs->arcade(throttle, turn);

		if (pros::millis() - last_update >= 1000)
		{
			WinLib::Pose pose = WinLib::getPose(false); 
			printf("getPose() = (%.1f, %.1f, %.1f)\n", pose.x, pose.y, pose.theta);
			last_update = pros::millis(); 
		}

		// ace::Ctr(); 
		// dr4b::Ctr(); 

	
		/*if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
		{
			printf("Cascade's rotation: %.2f\n\n", ace::cascade.get_position() );
		}*/

		pros::delay(10); 
	}
}