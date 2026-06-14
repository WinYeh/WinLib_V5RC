#pragma once
#include "api.h"                // IWYU pragma: keep

namespace WinLib 
{
    /**
    * @brief A namespace representing the size of omniwheels.
    */
    namespace Omniwheel 
    {
        constexpr float NEW_2 = 2.125;
        constexpr float NEW_275 = 2.75;
        constexpr float OLD_275 = 2.75;
        constexpr float NEW_275_HALF = 2.744;
        constexpr float OLD_275_HALF = 2.74;
        constexpr float NEW_325 = 3.25;
        constexpr float OLD_325 = 3.25;
        constexpr float NEW_325_HALF = 3.246;
        constexpr float OLD_325_HALF = 3.246;
        constexpr float NEW_4 = 4;
        constexpr float OLD_4 = 4.18;
        constexpr float NEW_4_HALF = 3.995;
        constexpr float OLD_4_HALF = 4.175;
    } // namespace Omniwheel

    /**
    * @brief A class of tracking wheels used for odometry.
    */
    class TrackingWheel 
    {
        private:
            pros::Rotation *encoder; // pointer to the encoder object
            float diameter;          // in mm (converted from inches in the constructor)
            float offset;            // in mm, positive is forward

        public:
        /**
        * @brief Construct a new Tracking Wheel object
        *
        * @param encoder pointer to the encoder object
        * @param diameter diameter of the wheel in inches (automatically converted to mm later in the constructor)
        * @param offset offset of the wheel from the center of the robot in mm,
        * positive is forward
        */
        TrackingWheel(pros::Rotation *encoder, float diameter, float offset);

        /**
        * @brief Get the offset of the tracking wheel from the center of the robot in mm
        *
        * @return float offset in mm
        */
        float getOffset(); 

        /**
        * @brief Get the diameter of the tracking wheel in inches by dividing by 25.4 to convert from mm
        *
        * @return float diameter in inches
        */
        float getDiameter();

        /**
        * @brief Get the distance traveled by the tracking wheel in milimeters(mm)
        *
        * @return float distance traveled in mm
        */
        float getDistanceTraveled();

        /**
        * @brief Reset the rotational units of the tracking wheel to 0
        *
        */
        void reset();
    };


    /**
 * @brief class containing the sensors used for odometry
 */
    class OdomSensors 
    {
        public:
            /**
            * Sensors used for odometry. 
            * The tracking wheels are used to track the position of the robot. 
            * The IMU is used to track the heading of the robot.
            * If the robot uses more than 1 tracking wheel for vertical or horizontal tracking, 
            * please consider adding the additional tracking wheel as a pointer in the class and setting it to nullptr if not used.
            */   
            TrackingWheel* vertical;
            TrackingWheel* horizontal;
            pros::Imu* imu;

            /**
            * The sensors are stored in a class so that they can be easily passed to the chassis class
            * The variables are pointers so that they can be set to nullptr if they are not used
            * Otherwise the chassis class would have to have a constructor for each possible combination of sensors
            *
            * @param vertical pointer to the first vertical tracking wheel
            * @param horizontal pointer to the first horizontal tracking wheel
            * @param imu pointer to the IMU
            */
            OdomSensors(TrackingWheel* vertical, TrackingWheel* horizontal, pros::Imu* imu);
    };
} // namespace WinLib