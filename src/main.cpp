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
	setActiveChassis(ace::chassis);

	pros::lcd::initialize();
	pros::lcd::set_text(1, "Welcome to WinLib (v0.0.1).");
	printf("project initilized---\n");
}

void disabled() 
{

}

void competition_initialize() 
{
	// test::rot_V.set_reversed(true);   // vertical wheel is wired reversed; flip it so driving forward reads + 
    Chs->calibrate(true);
	WinLib::setPose(WinLib::Pose(0, 0, 0));   // ensure odom starts with a known pose (x=0, y=0, heading=0)
	ace::cascade.tare_position(); 
	printf ("chassis calibrated---\n");
}

void autonomous() 
{
	printf("auton begins---\n");

	/*
	Chs->moveToPoint(0, 600, 3000, {.maxSpeed = 12.0});
	{
		WinLib::Pose p = WinLib::getPose(false);
		printf("after moveToPoint(0, 600): (%.0f, %.0f, %.0f)\n", p.x, p.y, p.theta);
	}

	Chs->moveToPoint(0, 0, 3000, {.forwards = -1, .maxSpeed = 12.0});
	pros::delay(500); 
	{
		WinLib::Pose p = WinLib::getPose(false);
		printf("after moveToPoint(0, 0): (%.0f, %.0f, %.0f)\n", p.x, p.y, p.theta);
	}
	*/

	Chs->moveToPose(-600, 600, 270, 4000, {.lead = 0.3, .maxSpeed = 12., .minSpeed = 1.5, .earlyExitRange = 20});

	WinLib::Pose pose = WinLib::getPose(false);
	printf("after moveToPose(-600, 600, 270): (%.1f, %.1f, %.1f)\n", pose.x, pose.y, pose.theta);
}

void opcontrol() 
{
	competition_initialize(); // for testing purposes
	autonomous();		// for testing purposes 
	float last_update = pros::millis();

	// float targetHead = 180; 
	while (true)
	{	
		// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) 
		// {
			/* 
			// Angular Gain Adjustment
			Chs->turnBy(45, 2000, {.maxSpeed = 12.0, .minSpeed = 2.0, .earlyExitRange = 2});
			// targetHead = (targetHead == 180) ? 0 : 180;
			pros::delay(1000);	
			WinLib::Pose pose = WinLib::getPose(false);
			printf("Pose = (%.1f, %.1f, %.1f)\n", pose.x, pose.y, pose.theta); 
			printf("Heading = %.1f\n", WinLib::getHeading() );
			*/
		
			// Lateral Gain Adjustment
			// Chs->moveFor(1800, 0, 2000, {.maxSpeed = 12.0, .minSpeed = 2.5, .earlyExitRange = 10});
			// Chs->moveFor(-1800, 0, 2000, {.maxSpeed = 12.0, .minSpeed = 2.5, .earlyExitRange = 10});
		// } 

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