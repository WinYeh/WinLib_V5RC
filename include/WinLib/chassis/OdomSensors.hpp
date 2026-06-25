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
     * @brief CustomIMU — a calibrated wrapper around pros::IMU.
     *
     * Every physical V5 IMU has a tiny manufacturing bias in how it reports
     * rotation. A "perfect" 360° turn might read as 359.6° on one chip and
     * 360.5° on another. Over a 90-second match that bias compounds into
     * degrees of heading error, which becomes inches of position error in odom.
     *
     * The fix is empirical: spin the robot a known number of full turns on a
     * flat surface, read what the IMU reports, and bake the ratio into the
     * sensor itself. After that every consumer (odom, turnToHeading,
     * boomerang) gets the corrected number for free.
     *
     *   scalar = trueRotation / measuredRotation
     *
     * Example: 10 full turns = 3600°. IMU reports 3559°. scalar ≈ 1.01152.
     *
     * @b Important — why this is reached through a `CustomIMU*`, not a
     * `pros::Imu*`:
     *   pros::IMU::get_rotation() is NOT virtual in the PROS base. A call
     *   through `pros::Imu*` would skip this override and return the raw
     *   (unscaled) value. So OdomSensors below stores a `CustomIMU*` — that
     *   way the scalar always takes effect. Pass scalar = 1.0 for an
     *   uncalibrated IMU (same behavior as the base class).
     *
     * @b Example
     * @code {.cpp}
     * // in config.cpp:
     * WinLib::CustomIMU imu1(15, 1.01152008991);  // port 15, calibrated
     * WinLib::CustomIMU imu2(0,  1.0);            // port 0,  uncalibrated
     * @endcode
     */
    class CustomIMU : public pros::IMU
    {
        public:
            CustomIMU(int port, double scalar)
                : pros::IMU(port),
                  m_port(port),
                  m_scalar(scalar) {}

            // Unbounded total rotation in degrees, multiplied by the
            // calibration scalar. Calls the C API directly to avoid recursing
            // into the base if it ever becomes virtual.
            double get_rotation() const {
                return pros::c::imu_get_rotation(m_port) * m_scalar;
            }

        private:
            const int    m_port;
            const double m_scalar;
    };


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
     * @brief Which odometry algorithm the tracking task runs.
     *
     * TW2 — vertical + horizontal tracking wheels measure the
     *   forward and sideways step; heading comes from the IMU. Most accurate,
     *   but needs both wheels.
     * VPD — a single vertical tracking wheel measures the
     *   forward step; heading is derived from the drivetrain left/right encoder
     *   difference and fused with the IMU (see OdomSensors::imuTrust). No
     *   horizontal wheel, so sideways motion is assumed zero.
     */
    enum class OdomMode
    {
        TW2,
        VPD
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
            *
            * The IMU is stored as a `CustomIMU*` (not `pros::Imu*`) so that
            * the calibration scalar in CustomIMU::get_rotation() actually
            * takes effect — see the note on CustomIMU above.
            */
            TrackingWheel* vertical;
            TrackingWheel* horizontal;
            CustomIMU*     imu;

            /**
            * mode     — which odometry algorithm the tracking task runs.
            * imuTrust — for VPD only: the IMU's weight in
            *            the heading blend, 0..1. 1.0 = trust the IMU completely,
            *            0.0 = pure drivetrain heading (also the effective behavior
            *            when imu == nullptr). Ignored by TW2.
            */
            OdomMode mode;
            float    imuTrust;

            /**
            * The sensors are stored in a class so that they can be easily passed to the chassis class
            * The variables are pointers so that they can be set to nullptr if they are not used
            * Otherwise the chassis class would have to have a constructor for each possible combination of sensors
            *
            * @param vertical pointer to the first vertical tracking wheel
            * @param horizontal pointer to the first horizontal tracking wheel
            * @param imu pointer to the IMU (must be a CustomIMU — pass
            *            scalar = 1.0 if you don't have calibration data yet)
            * @param mode which odom algorithm to run (default TW2,
            *             so existing setups compile and behave unchanged)
            * @param imuTrust IMU weight in the drivetrain-mode heading blend (0..1)
            */
            OdomSensors(TrackingWheel* vertical, TrackingWheel* horizontal, CustomIMU* imu,
                        OdomMode mode     = OdomMode::TW2,
                        float    imuTrust = 0.98f);
    };
} // namespace WinLib
