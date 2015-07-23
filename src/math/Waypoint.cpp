#include "Waypoint.h"
#ifndef STAND_ALONE_MATH
    #include "comms/Protocol.h"
#else
    #include <stdint.h>
#endif
float Waypoint::getAltitude(){
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}
float Waypoint::getApproachSpeed(){
    return ((double)extra)/((double)Protocol::U16_FIXED_FACTOR);
}
