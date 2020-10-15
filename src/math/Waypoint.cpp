#include "Waypoint.h"

#ifndef STAND_ALONE_TEST
#include "comms/Protocol.h"
float Waypoint::getAltitude(){
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}
float Waypoint::getApproachSpeed(){
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}
Waypoint::Components Waypoint::headingComponents(const Waypoint& target) const {
    float aRlat = this->radLatitude();
    float aRlng = this->radLongitude();
    float bRlat = target.radLatitude();
    float bRlng = target.radLongitude();
    Waypoint::Components result;
    result.y = sin(bRlng - aRlng) * cos(bRlat);
    result.x = cos(aRlat)*sin(bRlat) - sin(aRlat)*cos(bRlat)*cos(bRlng - aRlng);
    return result;
}
float Waypoint::headingTo(const Waypoint& target) const {
    auto cmps = headingComponents(target);
    return toDeg( atan2(cmps.y,cmps.x) );
}
float Waypoint::distanceTo(const Waypoint& target) const {
    float aRlat  = this->radLatitude();
    float aRlng  = this->radLongitude();
    float bRlat  = target.radLatitude();
    float bRlng  = target.radLongitude();
    float sinlat = sin((aRlat - bRlat)/2.);
    float sinlng = sin((aRlng - bRlng)/2.);
    float chord  = sinlat*sinlat + sinlng*sinlng*cos(aRlat)*cos(bRlat);
    return 2. * Units::EARTH_RAD * atan2( sqrt(chord), sqrt(1.-chord) );
}

//degrees,miles

Waypoint Waypoint::extrapolate(float bearing, float distance) const 
{
    // for small distances the error from earth's curvature is less than
    // the error in the floating point trig, so a quick rectilinear
    // approximation behaves better
    if(distance < 2.0 /*miles*/){
        float dy = cos(toRad(bearing)) * distance;
        float dx = sin(toRad(bearing)) * distance;
        float dlat = dy*(360.0 /  Units::EARTH_CIRC);
        float dlon = dx*(360.0 / (Units::EARTH_CIRC*cos(radLatitude())));
        return Waypoint(degLatitude() +dlat,
                        degLongitude()+dlon,
                        Units::DEGREES,
                        extra);
    }
    // Full great circle curve algorithm
    float rlat, rlng;
    float pRlat = radLatitude();
    float pRlng = radLongitude();
    rlat = asin(sin(pRlat)*cos(distance/Units::EARTH_RAD) +
                cos(pRlat)*sin(distance/Units::EARTH_RAD)*cos(toRad(bearing)));
    rlng = pRlng+atan2(
              (sin(toRad(bearing))*sin(distance/Units::EARTH_RAD)*cos(pRlat)),
              (cos(distance/Units::EARTH_RAD)-sin(pRlat)*sin(rlat))
           );
    return Waypoint(rlat, rlng, Units::RADIANS, extra);
}
#endif
