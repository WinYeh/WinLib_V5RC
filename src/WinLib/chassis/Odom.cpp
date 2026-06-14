// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include "pros/rtos.hpp"
#include "WinLib/util.hpp"
#include "WinLib/chassis/Odom.hpp"
#include "WinLib/chassis/OdomSensors.hpp"
#include "config.h"   // for the global `chassis` — odom reads its OdomSensors through it.
                      // Single source of truth: `chassis.odomSensors` lives in config.cpp.

// tracking thread
pros::Task* trackingTask = nullptr;

// global variables (odom's running state — calculated each tick).
// NOTE: there is no `odomSensors` here on purpose. The sensors live on the
// Chassis (`chassis.odomSensors`), and update() reads through that. Keeping a
// local copy here would just create a second source of truth to keep in sync.
WinLib::Pose odomPose(0, 0, 0); // the pose of the robot
WinLib::Pose odomSpeed(0, 0, 0); // the speed of the robot
WinLib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot

float prevVertical = 0;
float prevHorizontal = 0;
float prevImu = 0;

WinLib::Pose WinLib::getPose(bool radians) {
    if (radians) 
        return odomPose;
    else 
        return WinLib::Pose(odomPose.x, odomPose.y, RadToDeg(odomPose.theta));
}

void WinLib::setPose(WinLib::Pose pose, bool radians) {
    if (radians) 
        odomPose = pose;
    else 
        odomPose = WinLib::Pose(pose.x, pose.y, DegToRad(pose.theta));
}

WinLib::Pose WinLib::getSpeed(bool radians) {
    if (radians) 
        return odomSpeed;
    else 
        return WinLib::Pose(odomSpeed.x, odomSpeed.y, RadToDeg(odomSpeed.theta));
}

/* 
   Kalman filter to be added 
   (there is inially getLocalSpeed and estimatePose func in this space here, 
   but they are not implemented by genesis and therefore are deleted)  
*/

void WinLib::update()
{
    // Pull sensor pointers from the chassis (the single source of truth).
    // If chassis hasn't been constructed yet (very early boot), or any side
    // is unwired, the null-guards below handle it gracefully.
    WinLib::TrackingWheel* verticalWheel   = chassis.odomSensors.vertical;
    WinLib::TrackingWheel* horizontalWheel = chassis.odomSensors.horizontal;
    pros::Imu*             imu             = chassis.odomSensors.imu;

    // may add particle filter in the future
    // get the current sensor values
    float verticalRaw = 0;
    float horizontalRaw = 0;
    float imuRaw = 0;
    if (verticalWheel != nullptr)
        verticalRaw = verticalWheel->getDistanceTraveled();
    if (horizontalWheel != nullptr)
        horizontalRaw = horizontalWheel->getDistanceTraveled();
    if (imu != nullptr)
        imuRaw = DegToRad(imu->get_rotation());

    // calculate the change in sensor values
    float deltaVertical = verticalRaw - prevVertical;
    float deltaHorizontal = horizontalRaw - prevHorizontal;
    float deltaImu = imuRaw - prevImu;

    // trackingWheel offsets setttings
    float horizontalOffset = 0;
    float verticalOffset = 0;
    if (verticalWheel != nullptr) 
        verticalOffset = verticalWheel->getOffset();
    if (horizontalWheel != nullptr)
        horizontalOffset = horizontalWheel->getOffset();

    // calculate the heading of the robot
    // Heading source: the Inertial Sensor (IMU).
    // A single horizontal tracking wheel CANNOT measure heading on its own — its reading
    // mixes sideways movement with rotation, so the two can't be separated. The horizontal
    // wheel instead corrects for sideways drift later, in the localX term below.
    // (Dual parallel vertical wheels and drivetrain tracking were dropped: the former is
    //  uncommon in modern vex, the latter is out of scope for this lib.)
    float heading = odomPose.theta;
    if (imu != nullptr)
        heading += deltaImu;
    // else: no heading source available, heading remains unchanged
    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    // calculate change in x and y
    float deltaY = 0;
    float deltaX = 0;
    if (verticalWheel != nullptr) 
        deltaY = verticalRaw - prevVertical;
    if (horizontalWheel != nullptr) 
        deltaX = horizontalRaw - prevHorizontal;

    // update the previous sensor values
    prevVertical = verticalRaw;
    prevHorizontal = horizontalRaw;
    prevImu = imuRaw;

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0)    // prevent divide by 0
    { 
        localX = deltaX;
        localY = deltaY;
    } 
    else 
    {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    // save previous pose
    WinLib::Pose prevPose = odomPose;

    // calculate global x and y
    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta = heading;

    // calculate speed
    odomSpeed.x = ema((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
    odomSpeed.y = ema((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / 0.01, odomSpeed.theta, 0.95);
}

void WinLib::init() {
    if (trackingTask == nullptr) 
    {
        trackingTask = new pros::Task 
        {
            [=] 
            {
                while (true) 
                {
                    update();
                    pros::delay(10);
                }
            }
        };
    }
}
