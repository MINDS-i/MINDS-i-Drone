#ifndef ORIENTATION_ENGINE_H
#define ORIENTATION_ENGINE_H

class InertialManager;
#include "input/InertialManager.h"
#include "math/Quaternion.h"
#include "math/Vec3.h"

/*
-Abstract Base Class for methods of sensor integration and state tracking
-Inheritors need not worry about data quality or origins, this is outside of
	the scope of this interface; Mean Square Error will be reported
-Inheritors will receive a normalized description of what the sensors are
	currently 'seeing' as roll rates around each axis and a quaternion
	of the obsolute state; It will use this to deduce a unified system state
-an option for "relativeYaw" is added when both rates (z) and state (Z) are
	given as input; this requires that only the rates be used for yaw data,
	and the Z quaternion not affect state yaw estimates.
*/

class OrientationEngine{
public:
	virtual void update(InertialManager* sensors)=0;
	virtual void updateRate(Vec3 z, float MSE)=0; //pry apparent angle
	virtual void updateAttitude(Quaternion Z, float MSE)=0;
	virtual Quaternion	getAttitude()=0;
	virtual Quaternion	getLastAttitude()=0;
	virtual Quaternion	getRateQuaternion()=0;
	virtual Vec3		getRate()=0;
	virtual float getRollRate()=0;
	virtual float getPitchRate()=0;
	virtual float getYawRate()=0;
};
#endif
