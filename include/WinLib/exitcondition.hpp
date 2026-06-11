#pragma once

namespace WinLib {
class ExitCondition 
{
    public:
        /**
         * @brief Create a new Exit Condition
         *
         * @param range the range where the countdown is allowed to start
         * @param time how much time to wait while in range before exiting
         *
         * @b Example
         * @code {.cpp}
         * // create a new exit condition that will exit if the input is within 0.1 of the target for 1000ms
         * ExitCondition ec(0.1, 1000);
         * @endcode
         */
        ExitCondition(const float range, const int time);
        /**
         * @brief whether the exit condition has been met
         *
         * @return true exit condition met
         * @return false exit condition not met
         *
         * @b Example
         * @code {.cpp}
         * // check if the exit condition has been met
         * if (ec.getExit()) {
         *     // do something
         * }
         * @endcode
         */
        bool getExit();
        /**
         * @brief update the exit condition
         *
         * @param input the input for the exit condition
         * @return true exit condition met
         * @return false exit condition not met
         *
         * @b Example
         * @code {.cpp}
         * // update the exit condition
         * // this is typically called in a loop
         * while (!ec.getExit()) {
         *     // do something
         *     ec.update(input);
         * }
         * @endcode
         */
        bool update(const float input);
        /**
         * @brief reset the exit condition timer
         *
         * @b Example
         * @code {.cpp}
         * // reset the exit condition timer
         * ec.reset();
         * @endcode
         */
        void reset();
        /**
         * @brief change the allowed range mid-run
         *
         * Lets a route tighten/loosen its "good enough" window between movements
         * without constructing a new ExitCondition. Also resets the timer so the
         * new range gets a fresh countdown.
         *
         * @param newRange the new range in the same units as the input
         */
        void setRange(const float newRange);
        /**
         * @brief change the required time-in-range mid-run
         *
         * Same idea as setRange — useful when a particular phase of the auton
         * wants a snappier or slower settle. Also resets the timer.
         *
         * @param newTime the new time-in-range, in milliseconds
         */
        void setTime(const int newTime);
        /**
         * @brief read the current allowed range / time-in-range.
         *
         * Used by ControllerSettings to copy the ExitCondition's values into
         * its own flat float fields at construction time.
         */
        float getRange() const;
        int   getTime()  const;
    protected:
        float range;
        int time;
        int startTime = -1;
        bool done = false;
};
} // namespace WinLib