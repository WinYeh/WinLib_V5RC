#pragma once
#include "WinLib/pose.hpp"      // IWYU pragma: keep

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
        /**
         * @brief settled-by-velocity exit check (stateless).
         *
         * Returns true once the robot's measured speed has dropped to or below
         * `minVelocity` — i.e. the bot has slowed to a crawl and is effectively
         * stopped. Call it in a motion's while-loop predicate (true = exit),
         * OR'd with the normal update()/getExit() position exit: the motion then
         * ends when either the bot reaches the target OR it stops making progress.
         *
         * @note This predicate is STATELESS — it ignores this object's range /
         *       time / debounce state entirely. It's a single-tick check, not a
         *       debounced one. The position exit and the timeout it sits next to
         *       in the loop are its safety net. Don't add a debounce here to
         *       "match" update(); only do so if a real route bails out early
         *       because of one transient slow tick.
         * @note The caller supplies the velocity; this does NOT read sensors.
         *       Recommended source: drivetrain velocity — the average of
         *       leftMotors->get_actual_velocity() and rightMotors->get_actual_velocity().
         *       Pick one source per motion and stay consistent. `minVelocity` is
         *       tuned per axis (lateral vs. angular use different units).
         *
         * @param velocity    current measured speed (sign is ignored)
         * @param minVelocity speed at or below which the bot counts as settled
         * @return true once |velocity| <= minVelocity
         */
        bool hasReachedMinVel(float velocity, float minVelocity);
        /**
         * @brief heading-crossed exit check for boomerang-style moves (stateless).
         *
         * Returns true once `pose` has crossed the line that runs perpendicular
         * to heading `theta` and passes through `target`. This is what lets a
         * boomerang moveToPose end cleanly: the carrot point pulls the bot along
         * a curve, so it may never land exactly on `target` and a plain distance
         * check could spin forever. Crossing that perpendicular line means "I'm
         * level with the target along my travel direction — close enough, stop."
         *
         * @note Like hasReachedMinVel, this is STATELESS — it ignores this
         *       object's range / time / debounce state. Call it inline in the
         *       while-loop, no reset()/update() needed.
         *
         * @param pose      the robot's current pose
         * @param target    the pose being driven to
         * @param theta     heading (in degrees) the crossing line is aligned to
         * @param tolerance shifts the line forward along `theta`. Positive
         *                  tolerance ends the move earlier (the bot crosses
         *                  sooner); defaults to 0.
         * @return true once `pose` has crossed the perpendicular line through `target`
         */
        bool hasCrossedTarget(Pose pose, Pose target, float theta, float tolerance = 0);
    protected:
        float range;
        int time;
        int startTime = -1;
        bool done = false;
};
} // namespace WinLib