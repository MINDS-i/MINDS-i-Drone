#include "Waypoint.h"

#ifndef STAND_ALONE_TEST
#include "comms/DroneProtocol.h"
float Waypoint::getAltitude(){
    return ((double)extra)/((double)DroneProtocol::U16_FIXED_FACTOR);
}
float Waypoint::getApproachSpeed(){
    return ((double)extra)/((double)DroneProtocol::U16_FIXED_FACTOR);
}
#endif
