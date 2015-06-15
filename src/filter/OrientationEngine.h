#ifndef ORIENTATION_ENGINE_H
#define ORIENTATION_ENGINE_H

class InertialManager;
#include "input/InertialManager.h"
#include "math/Quaternion.h"
#include "math/Vec3.h"

/*
-Abstract Base Class for methods of sensor integration and state tracking
-Inheritors will use the data from the given InertialManager to track
	orientation
-Inheritors report the attitude in quaternion form, and the rotation rates
	as instantanious rotational velocities about the internal frame
	("gyroscope" values)
-When calibrate(true) is called, the craft is going to be in a steady position
	(usually also arming the ESC's) and the filter can use that information
	to get a finer calibration on the sensors
*/

class OrientationEngine{
public:
	virtual void update(InertialManager& sensors)=0;
	virtual void calibrate(bool mode)=0;
	virtual Quaternion getAttitude()=0;
	virtual Vec3       getRate()=0;
	virtual float getRollRate(){  return getRate()[1]; }
	virtual float getPitchRate(){ return getRate()[0]; }
	virtual float getYawRate(){   return getRate()[2]; }
	virtual float getRoll(){  return getAttitude().getRoll(); }
	virtual float getPitch(){ return getAttitude().getPitch();}
	virtual float getYaw(){   return getAttitude().getYaw(); }
};
#endif
