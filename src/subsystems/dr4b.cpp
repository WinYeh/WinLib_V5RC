#include "config.h"                    // IWYU pragma: keep
#include "pros/misc.h"
#include "subsystems/dr4b.hpp"

namespace dr4b
{
    void init()
    {
        // Set brake modes once here so Hold() only needs to brake() each tick.
            }

    namespace Wrist
    {
        void CW()
        {
            wrist.move_velocity(127);
        }
        void CCW()
        {
            wrist.move_velocity(-127);
        }
        void Hold()
        {
            lifter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            wrist.brake();   // brake mode (HOLD) was set once in dr4b::init()
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

    namespace Lifter
    {
        void Up()
        {
            lifter.move_velocity(100);
        }
        void Down()
        {
            lifter.move_velocity(-100);
        }
        void Hold()
        {
            wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            lifter.brake(); // brake mode (HOLD) was set once in dr4b::init()
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
        dr4b::Claw::Ctr(); 
        dr4b::Wrist::Ctr();
        dr4b::Lifter::Ctr(); 
    }
}