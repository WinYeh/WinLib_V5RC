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

    float integral = 0;
    float prevError = 0;

    public:
        /**
         * @brief Construct a new PID
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         *
         * @b Example
         * @code {.cpp}
         * // create a PID
         * PID pid(5, // kP
         *         0.01, // kI
         *         20, // kD); 
         * @endcode
         */
        PID(float kP, float kI, float kD);

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
         * @brief Compute the PID of "Lateral"
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
        float compute_l(float error);

        /**
         * @brief Compute the PID of "Angular"
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
        float compute_a(float error);

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
};

}   // namespace WinLib