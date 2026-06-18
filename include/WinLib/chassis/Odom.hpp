#pragma once

#include "WinLib/chassis/OdomSensors.hpp"       // IWYU pragma: keep
#include "WinLib/pose.hpp"

namespace WinLib 
{
    /**
    * @brief Get the pose of the robot
    *
    * @param radians true for theta in radians, false for degrees. False by default
    * @return Pose
    */
    Pose getPose(bool radians = false);
    /**
    * @brief Get the robot's heading, wrapped into a compass range.
    *
    * Convenience wrapper over getPose().theta. odom's theta is unbounded — it
    * keeps counting past 360° like a trip odometer — so this wraps it into
    * [0, 360) degrees (or [0, 2*pi) radians) the way a compass reads. The value
    * is the calibrated, odom-tracked heading, so it includes any setPose
    * corrections (DSR resets, your starting heading).
    *
    * @param radians true for [0, 2*pi) radians, false for [0, 360) degrees. False by default
    * @return float heading
    */
    float getHeading(bool radians = false);
    /**
    * @brief Set the Pose of the robot
    *
    * @param pose the new pose
    * @param radians true if theta is in radians, false if in degrees. False by default
    */
    void setPose(Pose pose, bool radians = false);
    /**
    * @brief Get the speed of the robot
    *
    * @param radians true for theta in radians, false for degrees. False by default
    * @return WinLib::Pose
    */
    Pose getSpeed(bool radians = false);
    
    /* 
        Kalman filter to be added 
        (there is inially getLocalSpeed and estimatePose func in this space here, 
        but they are not implemented by genesis and therefore is deleted)  
    */

    /**
    * @brief Update the pose of the robot
    *
    */
    void OdomUpdate();
    /**
    * @brief Initialize the odometry system
    *
    */
    void OdomInit();
} // namespace WinLib
