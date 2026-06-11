#pragma once
#define M_e 2.718281828
#include <vector>
#include "WinLib/pose.hpp"
#include <cmath>

namespace WinLib
{

/**
 * @brief x^e function 
 *
 * Calculates the value of pow(input, e) 
 * @param input the x as input  
 * @return the value of input^e
 */
double xEe_Func(double input); 

/**
 * @brief AngularDirection
 *
 * When turning, the user may want to specify the direction the robot should turn in.
 * This enum class has 3 values: CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, and AUTO
 * AUTO will make the robot turn in the shortest direction, and will be the most used value
 */
enum class AngularDirection 
{
    CW_CLOCKWISE,           /** turn clockwise */
    CCW_COUNTERCLOCKWISE,   /** turn counter-clockwise */
    AUTO                    /** turn in the direction with the shortest distance to target */
};

/**
 * @brief convert degree to radian
 */
double DegToRad(double angleInDeg);

/**
 * @brief convert radian to degree
 */
double RadToDeg(double angleInRad);

/**
 * @brief Calculate Angle Error
 *
 * When turning, the angleError func calculates the error of the angle based on the assigned Angular Direction
 * CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, and AUTO
 * AUTO will make the robot turn in the shortest direction, and will be the most used value
 * @param target target of the Heading the robot should turn to 
 * @param curr current Heading of the robot
 * @param radians whether the units of target or curr is in radians 
 * @param direction assigned direction to turn 
 */
double angleError(double target, double curr, bool radians, AngularDirection direction);

/**
 * @brief Return the average of a vector of numbers
 *
 * @param values
 * @return float
 *
 * @b Example
 * @code {.cpp}
 * std::vector<float> values = {1, 2, 3, 4, 5};
 * avg(values); // returns 3
 * @endcode
 */
float avg(std::vector<float> values);

/**
 * @brief Exponential moving average
 * It places more weight on the most recent data points and exponentially less on older ones.
 * Main purpose: to reduce & filter out the background noice from sensor 
 * formula: (current * smooth) + (previous * (1 - smooth) )
 *
 * @param current current measurement
 * @param previous previous output
 * @param smooth smoothing factor (0-1). 1 means no smoothing, 0 means no change
 * @return float - the smoothed output
 *
 * @b Example
 * @code {.cpp}
 * ema(10, 0, 0.5); // returns 5
 * @endcode
 */
float ema(float current, float previous, float smooth);

/** for curvature motions (probably can be deleted)
 * @brief Get the signed curvature of a circle that intersects the first pose and the second pose
 *
 * This is a very niche function that is only used in Pure Pursuit and Boomerang. It calculates the curvature of a
 * circle that is tangent to the first pose and intersects the second pose. It's also signed to indicate whether the
 * robot should turn clockwise or counter-clockwise to get to the second pose
 *
 * @note The circle will be tangent to the theta value of the first pose
 * @note The curvature is signed. Positive curvature means the circle is going clockwise, negative means
 * counter-clockwise
 * @note Theta has to be in radians and in standard form. That means 0 is right and increases counter-clockwise
 *
 * @param pose the first pose
 * @param other the second pose
 * @return float curvature
 *
 * @b Example
 * @code {.cpp}
 * Pose pose = {0, 0, 0};
 * Pose other = {0, 10, 0};
 * float curvature = getCurvature(pose, other);
 * @endcode
*/
float getCurvature(Pose pose, Pose other);


} // namespace WinLib 