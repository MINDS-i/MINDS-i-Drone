#include "Waypoint.h"

#ifndef STAND_ALONE_TEST
#include "comms/Protocol.h"
float Waypoint::getAltitude(){
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}
float Waypoint::getApproachSpeed(){
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}
#endif
