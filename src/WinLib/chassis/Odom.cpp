// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include <cmath>   // std::isfinite — used by the glitch guard in both update modes
#include "pros/rtos.hpp"
#include "WinLib/util.hpp"
#include "WinLib/chassis/Odom.hpp"
#include "WinLib/chassis/OdomSensors.hpp"
#include "config.h"   // for the global `Chs` — odom reads its OdomSensors through it.
                      // `Chs` points at the active robot's chassis (set in main.cpp's
                      // initialize()). Single source of truth: `Chs->odomSensors`.

// tracking thread
pros::Task* trackingTask = nullptr;

// global variables (odom's running state — calculated each tick).
// NOTE: there is no `odomSensors` here on purpose. The sensors live on the
// active Chassis (`Chs->odomSensors`), and update() reads through that. Keeping
// a local copy here would just create a second source of truth to keep in sync.
WinLib::Pose odomPose(0, 0, 0); // the pose of the robot
WinLib::Pose odomSpeed(0, 0, 0); // the speed of the robot
WinLib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot

float prevVertical = 0;
float prevHorizontal = 0;
float prevImu = 0;
float prevDriveLeft = 0;   // mm — left/right drivetrain travel, used only by the
float prevDriveRight = 0;  // VPD mode

WinLib::Pose WinLib::getPose(bool radians) {
    if (radians)
        return odomPose;
    else
        return WinLib::Pose(odomPose.x, odomPose.y, RadToDeg(odomPose.theta));
}

float WinLib::getHeading(bool radians) {
    // odomPose.theta is unbounded (it accumulates past 360°), so wrap it into
    // [0, 360) in degrees, then convert to radians if asked. fmod can return a
    // negative value, so add a full turn back if it does.
    float heading = std::fmod(getPose(false).theta, 360.0f);
    if (heading < 0) heading += 360.0f;
    return radians ? DegToRad(heading) : heading;
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

// Shared integrator: take this tick's local step (localX sideways, localY
// forward, mm) and heading change (deltaHeading, radians) and fold them into the
// running global pose. Both odom modes build their local step their own way and
// hand it here, so the chord-at-average-heading projection (the arc result: the
// step points at theta_prev + dTheta/2) lives in exactly one place.
static void integrateStep(float localX, float localY, float deltaHeading)
{
    // the step is a chord pointed at the AVERAGE heading over the tick
    float avgHeading = odomPose.theta + deltaHeading / 2;

    // remember the old pose for the speed estimate below
    WinLib::Pose prevPose = odomPose;

    // project the local step into field X/Y and accumulate. Heading 0 points
    // along +Y in this convention, so sin/cos look swapped vs the textbook —
    // same math, rotated 90°.
    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta += deltaHeading;

    // speed = change / tick (0.01 s), smoothed with an exponential moving average
    odomSpeed.x     = WinLib::ema((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
    odomSpeed.y     = WinLib::ema((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
    odomSpeed.theta = WinLib::ema((odomPose.theta - prevPose.theta) / 0.01, odomSpeed.theta, 0.95);
}

// --- TW2 mode (the original algorithm) ---
static void OdomUpdate_TW2()
{
    // Pull sensor pointers from the active chassis (the single source of truth).
    // If any side is unwired, the null-guards below handle it gracefully.
    WinLib::TrackingWheel* verticalWheel   = Chs->odomSensors.vertical;
    WinLib::TrackingWheel* horizontalWheel = Chs->odomSensors.horizontal;
    WinLib::CustomIMU*     imu             = Chs->odomSensors.imu;

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
        imuRaw = WinLib::DegToRad(imu->get_rotation());

    // Glitch guard: a failed PROS read returns PROS_ERR_F (== INFINITY); any math
    // on it makes NaN, which would poison odomPose forever. Sanitize EACH read on
    // its own — replace a non-finite value with that sensor's last good reading so
    // its delta is 0 this tick — instead of bailing the whole update (which would
    // freeze ALL of odom, theta stuck at 0, the moment one sensor was unplugged).
    if (!std::isfinite(verticalRaw))   verticalRaw   = prevVertical;
    if (!std::isfinite(horizontalRaw)) horizontalRaw = prevHorizontal;
    if (!std::isfinite(imuRaw))        imuRaw        = prevImu;

    // trackingWheel offsets
    float horizontalOffset = 0;
    float verticalOffset = 0;
    if (verticalWheel != nullptr)
        verticalOffset = verticalWheel->getOffset();
    if (horizontalWheel != nullptr)
        horizontalOffset = horizontalWheel->getOffset();

    // Heading change comes from the IMU only in this (two-tracking-wheel) mode.
    // A single horizontal tracking wheel CANNOT measure heading on its own — its
    // reading mixes sideways movement with rotation, so the two can't be
    // separated. The horizontal wheel instead corrects for sideways drift in the
    // localX term below.
    float deltaHeading = (imu != nullptr) ? (imuRaw - prevImu) : 0;

    // change in the tracking-wheel distances since last tick
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

    // offset-corrected local step (an offset wheel sweeps its own little arc when
    // the robot rotates; this removes that). deltaHeading == 0 avoids divide-by-0.
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0)
    {
        localX = deltaX;
        localY = deltaY;
    }
    else
    {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    // project the local step into the global pose (shared with the drivetrain mode)
    integrateStep(localX, localY, deltaHeading);
}

// --- VPD mode ---
// Forward step from the vertical tracking wheel (drivetrain average as a
// fallback); heading from the drivetrain left/right difference, fused with the
// IMU by trust; sideways assumed zero (no horizontal wheel). See OdomMode.
static void OdomUpdate_VPD()
{
    WinLib::TrackingWheel* verticalWheel = Chs->odomSensors.vertical;
    WinLib::CustomIMU*     imu           = Chs->odomSensors.imu;
    pros::MotorGroup*      leftMotors    = Chs->drivetrain.leftMotors;
    pros::MotorGroup*      rightMotors   = Chs->drivetrain.rightMotors;

    // read the drivetrain sides (motor degrees → mm of travel) and the sensors
    float driveLeft  = (leftMotors  != nullptr) ? Chs->degToMM(leftMotors->get_position())  : 0;
    float driveRight = (rightMotors != nullptr) ? Chs->degToMM(rightMotors->get_position()) : 0;
    float verticalRaw = (verticalWheel != nullptr) ? verticalWheel->getDistanceTraveled() : 0;
    float imuRaw      = (imu != nullptr) ? WinLib::DegToRad(imu->get_rotation()) : 0;

    // Glitch guard: a failed PROS read returns PROS_ERR_F (== INFINITY), and any
    // math on it makes NaN, which would poison odomPose forever. Sanitize EACH
    // read on its own — replace a non-finite value with that sensor's last good
    // reading, so its delta is 0 this tick. (An earlier version bailed the WHOLE
    // update when ANY read was bad; that froze ALL of odom — theta stuck at 0 —
    // the moment a single motor/sensor was unplugged. Per-sensor sanitizing
    // degrades gracefully: the healthy sensors keep updating the pose.)
    if (!std::isfinite(driveLeft))   driveLeft   = prevDriveLeft;
    if (!std::isfinite(driveRight))  driveRight  = prevDriveRight;
    if (!std::isfinite(verticalRaw)) verticalRaw = prevVertical;
    if (!std::isfinite(imuRaw))      imuRaw      = prevImu;

    // First tick: the drivetrain encoders are never tared, so seed the prev
    // values to the current readings and skip integrating to avoid a huge delta.
    static bool seeded = false;
    if (!seeded)
    {
        prevDriveLeft  = driveLeft;
        prevDriveRight = driveRight;
        prevVertical   = verticalRaw;
        prevImu        = imuRaw;
        seeded = true;
        return;
    }

    // changes since last tick
    float deltaLeft     = driveLeft   - prevDriveLeft;
    float deltaRight    = driveRight  - prevDriveRight;
    float deltaVertical = verticalRaw - prevVertical;
    float deltaImu      = imuRaw      - prevImu;

    // Heading change: drivetrain differential, fused with the IMU.
    //   deltaThetaDrive = (left - right) / trackWidth      [Image 2]
    // SIGN CAVEAT: (deltaLeft - deltaRight) must turn the SAME way the IMU
    // reports. Verify on the robot — turn a known direction with imuTrust = 0
    // and confirm the heading moves the right way; if it's backwards, swap to
    // (deltaRight - deltaLeft).
    float trackWidthMM    = Chs->drivetrain.trackWidth * 25.4f;
    float deltaThetaDrive = (trackWidthMM != 0) ? (deltaLeft - deltaRight) / trackWidthMM : 0;

    float deltaHeading = (imu != nullptr)
        ? WinLib::blendByTrust(deltaImu, deltaThetaDrive, Chs->odomSensors.imuTrust)
        : deltaThetaDrive;

    // Forward step: vertical tracking wheel preferred (it doesn't slip), else the
    // drivetrain average.
    float deltaY = (verticalWheel != nullptr) ? deltaVertical
                                              : (deltaLeft + deltaRight) / 2;

    // offset-correct the forward step; no horizontal wheel, so sideways = 0
    float verticalOffset = (verticalWheel != nullptr) ? verticalWheel->getOffset() : 0;
    float localX = 0;
    float localY;
    if (deltaHeading == 0)
        localY = deltaY;
    else
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);

    // project + accumulate (shared integrator)
    integrateStep(localX, localY, deltaHeading);

    // save prev
    prevDriveLeft  = driveLeft;
    prevDriveRight = driveRight;
    prevVertical   = verticalRaw;
    prevImu        = imuRaw;
}

void WinLib::OdomUpdate()
{
    // Cheap insurance: odom can't run until a chassis is chosen. In the normal
    // flow this never trips (initialize() sets Chs before calibrate() starts the
    // task), but it guards against anyone reordering that later.
    if (Chs == nullptr) return;

    // Dispatch to the configured algorithm. Both build a local step and hand it
    // to integrateStep, so they share the global-pose math.
    switch (Chs->odomSensors.mode)
    {
        case WinLib::OdomMode::TW2:
            OdomUpdate_TW2();
            break;
        case WinLib::OdomMode::VPD:
            OdomUpdate_VPD();
            break;
    }
}

void WinLib::OdomInit() {
    if (trackingTask == nullptr) 
    {
        trackingTask = new pros::Task 
        {
            [=] 
            {
                while (true) 
                {
                    WinLib::OdomUpdate();
                    pros::delay(10);
                }
            }
        };
    }
}
