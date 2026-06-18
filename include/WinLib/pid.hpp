#pragma once 

namespace WinLib
{

class PID
{
    protected:
    // gains
    float kP;
    float kI;
    float kD;
    // anti-windup deadband — integral only accumulates when |error| < windupRange.
    // 0 disables the check (always accumulate), which matches the old 3-arg behavior.
    float windupRange = 0;

    float integral = 0;
    float prevError = 0;

    // last computed P / I / D term contributions. compute() builds the output
    // as p_term + i_term + d_term and saves each here so debugPID can print the
    // breakdown while tuning. 0 until the first compute().
    float p_term = 0;
    float i_term = 0;
    float d_term = 0;

    public:
        /**
         * @brief Construct a new PID
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         * @param windupRange anti-windup deadband. If non-zero, the integral
         *        term only accumulates when |error| < windupRange. 0 disables
         *        the check. 0 by default.
         *
         * @b Example
         * @code {.cpp}
         * // create a PID with no anti-windup
         * PID pid(5,    // kP
         *         0.01, // kI
         *         20);  // kD
         * // create a PID with an anti-windup deadband of 3
         * // (integral only grows when |error| < 3)
         * PID pid(5, 0.01, 20, 3);
         * @endcode
         */
        PID(float kP, float kI, float kD, float windupRange = 0);

        /**
         * @brief Compute the PID
         *
         * @param error target minus position - AKA error
         * @return float output
         *
         * @b Example
         * @code {.cpp}
         * void opcontrol() {
         *     // create a PID
         *     PID pid(5, 0, 20);
         *     // give the pid a test input
         *     // the pid will then return an output
         *     float output = pid.update(10);
         * }
         * @endcode
         */
        float compute(float error); 

        /**
         * @brief reset integral, derivative, and prevTime
         *
         * @b Example
         * @code {.cpp}
         * void opcontrol() {
         *     // create a PID
         *     PID pid(5, 0, 20);
         *     // give the pid a test input
         *     // the pid will then return an output
         *     float output = pid.compute(10);
         *     // reset the pid
         *     pid.reset();
         * }
         * @endcode
         */
        void reset();

        /**
         * @brief fucntions to set the value of kP, kI, kD individually 
         *
         * @b Example
         * @code {.cpp}
         * void opcontrol() {
         *     // create a PID
         *     PID pid(5, 0, 20);
         *     // set kP
         *     pid.setkP(10.);
         *     // set kI
         *     pid.setkP(1.);
         *     // set kD
         *     pid.setkP(30.);
         * }
         * @endcode
         */
        void setkP(float newkP);
        void setkI(float newkI);
        void setkD(float newkD);
        void setWindupRange(float newWindupRange);

        /**
         * @brief read the current gains / anti-windup deadband.
         *
         * Used by ControllerSettings to copy the PID's values into its own
         * flat float fields at construction time.
         */
        float getkP() const;
        float getkI() const;
        float getkD() const;
        float getWindupRange() const;

        /**
         * @brief read the most recent P / I / D term contributions.
         *
         * Each is saved by compute() (output = p_term + i_term + d_term), so
         * they reflect the last compute() call. Used by debugPID to print the
         * term breakdown while tuning. 0 before the first compute().
         */
        float getPTerm() const;
        float getITerm() const;
        float getDTerm() const;
};


class linear_PID : public PID
{
    public:
        linear_PID(float kP, float kI, float kD, float windupRange = 0);
        float compute(float error);
};

class angular_PID : public PID
{
    public:
        angular_PID(float kP, float kI, float kD, float windupRange = 0);
        float compute(float error);
};

/**
 * @brief Asymptotic gain-schedule curve (PIDPlus-style).
 *
 * Maps a motion's size (the setpoint) to a kP value on a smooth S-curve:
 *   - small setpoint   -> kP near `initial` (snappy, precise final settling)
 *   - large setpoint   -> kP near `final`   (gentle ramp, less overshoot)
 *   - setpoint == knee  -> kP at the midpoint, (initial + final) / 2
 * `power` controls how sharply the curve bends around the knee.
 *
 * Formula: (final - initial) * |s|^power / (|s|^power + knee^power) + initial
 *
 * Movement code calls this ONCE at the start of a motion to pick a kP from how
 * far it has to move, then holds that kP for the whole motion.
 *
 * @param setpoint how far the motion is being asked to move. Pass the |error|
 *                 measured once at motion start. Must be in the SAME units as
 *                 `knee` (e.g. heading degrees for a turn, motor degrees for
 *                 moveFor).
 * @param initial  kP when setpoint is ~0
 * @param final    kP the curve approaches as setpoint grows large
 * @param knee     setpoint at which kP is the midpoint. Must be > 0.
 * @param power    transition sharpness (higher = sharper bend)
 * @return the scheduled kP
 */
float asymptoticGain(float setpoint, float initial, float final, float knee, float power);

}   // namespace WinLib