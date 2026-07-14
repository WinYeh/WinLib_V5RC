#include "config.h"                    // IWYU pragma: keep
#include "pros/misc.h"
#include "subsystems/ace.hpp"

namespace catherine
{
    namespace Intake 
    {
        void INtake()
        {
            intake.move_velocity(127);
        }
        void outake()
        {
            intake.move_velocity(-127);
        }
        void Hold()
        {
            intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            intake.brake();   
        }

        void Ctr()
        {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            {
                INtake();
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            {
                outake();
            }
            else
            {
                Hold();
            }
        }
    }

    namespace Wrist 
    {
        void Normal()
        {
            // PID to be implemented here
            intake.move_velocity(127);
        }
        void Flipped()
        {
            // PID to be implemented here
            intake.move_velocity(-127);
        }
        void Hold()
        {
            intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            intake.brake();   
        }

        void Ctr()
        {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            {
                Normal();
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            {
                Flipped();
            }
            else
            {
                Hold();
            }
        }
    }

    namespace Claw
    {
        void Open()
        {
            claw.set_value(true); 
        }

        void Close()
        {
            claw.set_value(false); 
        }

        bool opened = false; // track the state of the claw
        void Ctr()
        {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
            {
                claw.set_value(!opened);
                opened = !opened;   
            }
        }
    }

    namespace Cascade
    {
        void Up()
        {
            cascade.move_velocity(127);
        }
        void Down()
        {
            cascade.move_velocity(-127);
        }
        void Hold()
        {
            cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            cascade.brake(); 
        }

        void Ctr()
        {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
            {
                Up();
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            {
                Down();
            }
            else
            {
                Hold();
            }
        }
    }

    void Ctr()
    {
        Intake::Ctr(); 
		ace::Claw::Ctr(); 
        Wrist::Ctr(); 
		Cascade::Ctr();
    }
}