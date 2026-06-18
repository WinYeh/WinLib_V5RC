// opcontrol.cpp — driver-control methods of Chassis.
//
// Right now there's only one method, `arcade`, doing the simplest possible
// joystick → motors mapping. No drive curves, no deadband, no scaling.
// Drive curves are deferred to a separate file later (see CLAUDE.md
// § Deferred Decisions: Drive curves).

#include "WinLib/chassis/chassis.hpp"

using namespace WinLib;

/* ---- arcade ----
 *
 * Classic arcade-drive mixing:
 *   left  = throttle + turn
 *   right = throttle - turn
 *
 * Joystick analog values are in the range -127..127. We convert each side
 * to motor voltage in millivolts (max ±12000 mV = 12 V) and send via
 * pros::Motor::move_voltage. The conversion is linear:
 *   mV = joystickValue * 12000 / 127
 *
 * No clamping is needed on our side — pros::Motor::move_voltage clips any
 * input outside ±12000 mV to those bounds automatically. So full-forward +
 * full-turn (throttle=127, turn=127 → left=254) lands at the 12 V ceiling
 * cleanly.
 *
 * Null-guards on the motor groups in case a side is unwired during testing.
 */
void Chassis::arcade(float throttle, float turn) 
{
    throttle *= fabs(throttle) > 5;
    turn *= fabs(turn) > 5; 
    float left  = throttle + turn;
    float right = throttle - turn;
    move_voltage(left * 12 / 127, right * 12 / 127);
}