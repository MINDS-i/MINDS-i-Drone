#include "Waypoint.h"

#ifndef STAND_ALONE_TEST
#include "comms/Protocol.h"
float Waypoint::getAltitude(){
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}
float Waypoint::getApproachSpeed(){
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}
Waypoint::Components Waypoint::headingComponents(Waypoint target) const {
    float aRlat = this->radLatitude();
    float aRlng = this->radLongitude();
    float bRlat = target.radLatitude();
    float bRlng = target.radLongitude();
    Waypoint::Components result;
    result.y = sin(bRlng - aRlng) * cos(bRlat);
    result.x = cos(aRlat)*sin(bRlat) - sin(aRlat)*cos(bRlat)*cos(bRlng - aRlng);
    return result;
}
float Waypoint::headingTo(Waypoint target) const {
    auto cmps = headingComponents(target);
    return toDeg( atan2(cmps.y,cmps.x) );
}
float Waypoint::distanceTo(Waypoint target) const {
    float aRlat  = this->radLatitude();
    float aRlng  = this->radLongitude();
    float bRlat  = target.radLatitude();
    float bRlng  = target.radLongitude();
    float sinlat = sin((aRlat - bRlat)/2.);
    float sinlng = sin((aRlng - bRlng)/2.);
    float chord  = sinlat*sinlat + sinlng*sinlng*cos(aRlat)*cos(bRlat);
    return 2. * Units::EARTH_RAD * atan2( sqrt(chord), sqrt(1.-chord) );
}
Waypoint
Waypoint::extrapolate(float bearing, float distance) const {//degrees,miles
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
