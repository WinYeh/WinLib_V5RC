#include "config.h"                    // IWYU pragma: keep
#include "pros/misc.h"
#include "subsystems/ace.hpp"

namespace ace
{
    namespace LadyBrown 
    {
        void CW()
        {
            ladybrown.move_velocity(127);
        }
        void CCW()
        {
            ladybrown.move_velocity(-127);
        }
        void Hold()
        {
            ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            ladybrown.brake();   
        }

        void Ctr()
        {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            {
                CW();
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            {
                CCW();
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
        ace::LadyBrown::Ctr(); 
		ace::Claw::Ctr(); 
		ace::Cascade::Ctr();
    }
}