#ifndef ORIENTATION_ENGINE_H
#define ORIENTATION_ENGINE_H

#include "math/quaternion.h"
#include "math/vector.h"

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
	virtual void update(math::vector3d z, math::quaternion Z,
						float rateMSE, float attitudeMSE,
						boolean relativeYaw);
	virtual void updateRate(math::vector3d z, float MSE); //rps euler angles
	virtual void updateAttitude(math::quaternion Z, float MSE); //quaternion
	virtual math::vector3d   getRate();
	virtual math::quaternion getAttitude();
	virtual math::quaternion getLastAttitude();
	virtual math::quaternion getRateQuaternion();
	virtual float getRoll();  //These functions output radians
	virtual float getPitch();
	virtual float getYaw();
	virtual float getRollRate();
	virtual float getPitchRate();
	virtual float getYawRate();
};
#endif
