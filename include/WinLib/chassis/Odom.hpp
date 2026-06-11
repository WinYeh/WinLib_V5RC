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
    void update();
    /**
    * @brief Initialize the odometry system
    *
    */
    void init();
} // namespace WinLib
