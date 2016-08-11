#ifndef GPS_H
#define GPS_H

#include "math/Waypoint.h"

/**
 * Interface for providing access to basic GPS information
 */
class GPS {
public:
    /**
     * The most recent Waypoint indicating the GPS's current location
     */
    virtual Waypoint getLocation() = 0;
    /**
     * The course over the ground in degrees, 0 = north, ccw positie
     */
    virtual float getCourse() = 0;
    /**
     * The most recent ground speed reading in miles per hour
     */
    virtual float getGroundSpeed() = 0;
    /**
     * An index that changes every time the data cached in this GPS object
     * is updated
     */
    virtual uint16_t dataIndex();
};

#endif
