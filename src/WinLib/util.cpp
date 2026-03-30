#include "WinLib/util.hpp"

double WinLib::xEe_Func(double input)
{
    return pow(std::fabs(input), M_e);  
}

double WinLib::angleError(double target, double curr, bool radians, WinLib::AngularDirection direction)
{
    double error =  target - curr;
    double max = radians ? 2 * M_PI : 360;
    switch (direction) 
    {
        case WinLib::AngularDirection::CW_CLOCKWISE:
            return error < 0 ? error + max : error;     // add max if sign does not match
        case WinLib::AngularDirection::CCW_COUNTERCLOCKWISE:
            return error > 0 ? error - max : error;     // subtract max if sign does not match
        default: 
            return error > 180 ? error - 360 : (error < -180 ? error + 360 : error);
    };
}

float WinLib::avg(std::vector<float> values) 
{
    float sum = 0;
    for (float value : values) { sum += value; }
    return sum / values.size();
}

/* for odom */
float WinLib::ema(float current, float previous, float smooth)
{
    return (current * smooth) + (previous * (1 - smooth));
}

/* for curvature motions (probably can be deleted) 
float WinLib::getCurvature(Pose pose, Pose other) 
{
    // calculate whether the pose is on the left or right side of the circle
    float side = WinLib::sgn(std::sin(pose.theta) * (other.x - pose.x) - std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}*/