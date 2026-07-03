#pragma once

#include <cstdint>

namespace WinLib 
{
class Timer 
{
    public:
        /**
         * @brief Construct a new Timer
         *
         * @note the timer will start counting down as soon as it is created
         * @note the timer constructor is non-blocking so code after it will be executed immediately
         * @note if the timer is constructed in a global scope, its behavior is undefined. You can
         *       call set() before using the timer if you absolutely need to construct it in a global scope
         *
         * @param time how long to wait, in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * @endcode
         */
        Timer(uint32_t time);
        /**
         * @brief Get the amount of time the timer was set to
         *
         * @return uint32_t time, in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // get the time the timer was set to
         * const uint32_t time = timer.getTimeSet(); // time = 1000
         * @endcode
         */
        uint32_t getTimeout();
        /**
         * @brief Get the amount of time left on the timer
         *
         * @return uint32_t time in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // delay for 300ms
         * pros::delay(300);
         * // get the time left on the timer
         * const uint32_t time = timer.getTimeLeft(); // time = 700
         * @endcode
         */
        uint32_t getTimeLeft();
        /**
         * @brief Get the amount of time passed on the timer
         *
         * @return uint32_t time in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // delay for 300ms
         * pros::delay(300);
         * // get the time passed on the timer
         * const uint32_t time = timer.getTimePassed(); // time = 300
         * @endcode
         */
        uint32_t getTimePassed();
        /**
         * @brief Get whether the timer is done or not
         *
         * @return true the timer is done
         * @return false the timer is not done
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // delay for 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * // delay for another 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = true
         * @endcode
         */
        bool isDone();
        /**
         * @brief Get whether the timer is paused or not
         *
         * @return true the timer is paused
         * @return false the timer is not paused
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // pause the timer
         * timer.pause();
         * // check if the timer is paused
         * bool paused = timer.isPaused(); // paused = true
         * // resume the timer
         * timer.resume();
         * // check if the timer is paused
         * paused = timer.isPaused(); // paused = false
         * @endcode
         */
        bool isPaused();
        /**
         * @brief Set the amount of time the timer should count down. Resets the timer
         *
         * @param time time in milliseconds
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // set the timer to wait for 2 seconds
         * timer.set(2000);
         * @endcode
         */
        void set(uint32_t time);
        /**
         * @brief reset the timer
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // delay for 500ms
         * pros::delay(500);
         * // reset the timer
         * timer.reset();
         * // delay for another 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * @endcode
         */
        void reset();
        /**
         * @brief pause the timer
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // pause the timer
         * timer.pause();
         * // delay for 2000ms
         * pros::delay(2000);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * @endcode
         */
        void pause();
        /**
         * @brief resume the timer
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // pause the timer
         * timer.pause();
         * // delay for 500ms
         * pros::delay(500);
         * // resume the timer
         * timer.resume();
         * // delay for another 500ms
         * pros::delay(500);
         * // check if the timer is done
         * const bool done = timer.isDone(); // done = false
         * @endcode
         */
        void resume();
        /**
         * @brief wait until the timer is done
         *
         * @b Example
         * @code {.cpp}
         * // create a timer that will wait for 1 second
         * Timer timer(1000);
         * // wait until the timer is done
         * timer.waitUntilDone();
         * std::cout << "done!" << std::endl;
         * @endcode
         */
        void waitUntilDone();
    private:
        uint32_t timeout;
        uint32_t lastTime;
        uint32_t timeWaited = 0;
        bool paused = false;

        /**
         * @brief Update the essetials variables of Timer per loop 
         *
         * @note updated variables - time, timewaited, lastTime 
         * @note it is a private func that should be called in func of the Timer class
         * @note this func is needed as it ensures that the time is accurate and precise 
         * @note it packages the following lines of code into a func used repeatedly in other Timer funcs
         *           const uint32_t time = pros::millis(); // get time from RTOS
         *           if (!paused) timeWaited += time - lastTime; // don't update if paused
         *           lastTime = time; // update last time
         *
         * @b Example
         * @code {.cpp}
         * uint32_t Timer::getTimeLeft() 
         * {
         *     this->update();  // updates timeout, timewaited, lastTime
         *     const int delta = timeout - timeWaited; // calculate how much time is left
         *     return (delta > 0) ? delta : 0; // return 0 if timer is done
         * }
        
         * @endcode
         */
        void update();
};
} // namespace WinLib