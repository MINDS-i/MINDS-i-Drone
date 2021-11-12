#include "Waypoint.h"

#ifndef STAND_ALONE_TEST
#include "comms/Protocol.h"
float Waypoint::getAltitude()
{
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}

float Waypoint::getApproachSpeed()
{
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}


//This isn't used much and should be updated to not use components (GPS_LOCAL seems better use)
Waypoint::Components Waypoint::headingComponents(const Waypoint& target) const 
{
    LOCAL_COORD lc;
    calc_haversine(&(this->m_gpsCoord),&(target.m_gpsCoord),&lc);
    Waypoint::Components result;
    result.y = lc.y;
    result.x = lc.x;
    return result;    
}

float Waypoint::headingTo(const Waypoint& target) const 
{
    LOCAL_COORD lc;
    calc_haversine(&m_gpsCoord,&target.m_gpsCoord,&lc);
    return lc.heading;
}

float Waypoint::distanceTo(const Waypoint& target) const 
{
    LOCAL_COORD lc;
    calc_haversine(&m_gpsCoord,&target.m_gpsCoord,&lc);
    return lc.distance/METERS_PER_MILE;
}

//degrees,miles

Waypoint Waypoint::extrapolate(float bearing, float distance) const 
{ 
    GPS_COORD gc;
    LOCAL_COORD lc;
    lc.distance=distance*METERS_PER_MILE;
    lc.heading=bearing;
    new_gps_float(&m_gpsCoord,&lc,&gc);

    return Waypoint(gc);
}
#endif
