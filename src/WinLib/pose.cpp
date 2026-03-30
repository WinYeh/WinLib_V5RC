#include "WinLib/pose.hpp"
#include <cmath>

WinLib::Pose::Pose(float x, float y, float theta) 
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

WinLib::Pose WinLib::Pose::operator+(const WinLib::Pose& other) const 
{
    return WinLib::Pose(this->x + other.x, this->y + other.y, this->theta);
}

WinLib::Pose WinLib::Pose::operator-(const WinLib::Pose& other) const 
{
    return WinLib::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float WinLib::Pose::operator*(const WinLib::Pose& other) const 
{ 
    return this->x * other.x + this->y * other.y; 
}

WinLib::Pose WinLib::Pose::operator*(const float& other) const 
{
    return WinLib::Pose(this->x * other, this->y * other, this->theta);
}

WinLib::Pose WinLib::Pose::operator/(const float& other) const 
{
    return WinLib::Pose(this->x / other, this->y / other, this->theta);
}

WinLib::Pose WinLib::Pose::lerp(WinLib::Pose other, float t) const 
{
    return WinLib::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float WinLib::Pose::distance(WinLib::Pose other) const 
{
    return std::hypot(this->x - other.x, this->y - other.y); 
}

float WinLib::Pose::angle(WinLib::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

WinLib::Pose WinLib::Pose::rotate(float angle) const 
{
    return WinLib::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string WinLib::format_as(const WinLib::Pose& pose) 
{
    // same logic as printf("Pose { x: %.2f, y: %.2f, theta: %.2f }, pose.x, pose.y, pose.theta);  
    return "Pose { x: " + std::to_string(pose.x) +
           ", y: "      + std::to_string(pose.y) +
           ", theta: "  + std::to_string(pose.theta) + " }";
}