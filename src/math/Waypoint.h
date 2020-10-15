#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "Arduino.h"
#include "SpatialMath.h"
#include <stdint.h>

class Waypoint
{

public:
	float lat, lng; //these are stored in degrees
	uint16_t extra; //8.8 fixed value used for altitude (air) or speed (ground)
	Waypoint():
			lat(0), lng(0), extra(0) {}
	Waypoint(float latitude, float longitude): //entry in degrees
			lat(latitude), lng(longitude), extra(0) {}
	Waypoint(float latitude, float longitude, Units::Rotation rad, uint16_t ex):
			lat(latitude), lng(longitude), extra(ex) {
		if(rad == Units::RADIANS){
			lat = toDeg(latitude);
			lng = toDeg(longitude);
		} //if not set to radians, they will stay initialized to degrees
	}
	void update(float latitude, float longitude){
		lat = latitude;
		lng = longitude;
	}
	void update(float latitude, float longitude, Units::Rotation rad){
		if(rad == Units::RADIANS){
			lat = toDeg(latitude);
			lng = toDeg(longitude);
		} else {
			lat = latitude;
			lng = longitude;
		}
	}
	float radLatitude() const  { return toRad(lat); }
	float radLongitude() const { return toRad(lng); }
	float degLatitude() const  { return lat; }
	float degLongitude() const { return lng; }
    struct Components{ float y, x; };
	/**
	 * Calculate the resulting vector as components for direct travel to
	 * the target waypoint. The resulting vector components are not normalized.
	 * x = North positive, south negative;
	 * y = East positive, west negative;
	 * @param  target The target waypoint
	 * @return        The vector of the shortest path
	 */
	Components headingComponents(const Waypoint& target) const;
	/**
	 * The heading in degrees from north ccw positive around the "down" vector
	 * to travel from this Waypoint to the target Waypoint on the shortest
	 * possible path
	 * @param  target Target Waypoint
	 * @return        Heading to travel in degrees, ccw from north
	 */
	float headingTo (const Waypoint& target) const;
	/**
	 * The distance in miles from this Waypoint to the target Waypoint
	 * @param  target Target Waypoint
	 * @return        Distance in miles
	 */
	float distanceTo(const Waypoint& target) const;
	/**
	 * calculate gps the waypoint after traveling `distance` on `bearing`
	 * from this waypoint
	 * @param  bearing  Direction of travel, degrees, north = 0, ccw positive
	 * around the down vector
	 * @param  distance Distance in miles
	 * @return          The resulting GPS location
	 */
	Waypoint extrapolate(float bearing, float distance) const;
	void setExtra(uint16_t alt){
		extra = alt;
	}
	uint16_t getExtra() const {
		return extra;
	}
#ifndef STAND_ALONE_TEST
	float getAltitude();
	float getApproachSpeed();
#endif
};

#endif
