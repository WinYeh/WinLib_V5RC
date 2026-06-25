#include "WinLib/chassis/OdomSensors.hpp"        // IWYU pragma: keep
using namespace WinLib;

/* TrackingWheel */ 
TrackingWheel::TrackingWheel(pros::Rotation *encoder, float diameter, float offset) 
    : encoder(encoder), diameter(diameter * 25.4f), offset(offset) {}

float TrackingWheel::getOffset() 
{ 
    return offset;     // in mm 
}

float TrackingWheel::getDiameter() 
{ 
    return diameter / 25.4f;   // in inches
}

float TrackingWheel::getDistanceTraveled()
{
    return (encoder->get_position() * M_PI * diameter) / 36000.0f; // convert degrees to distance in mm 
}

void TrackingWheel::reset() 
{
    encoder->reset_position();  // reset the encoder's position to 0
}


/* OdomSensors */
OdomSensors::OdomSensors(TrackingWheel* vertical, TrackingWheel* horizontal, CustomIMU* imu,
                         OdomMode mode, float imuTrust)
            : vertical(vertical), horizontal(horizontal), imu(imu),
              mode(mode), imuTrust(imuTrust) {}