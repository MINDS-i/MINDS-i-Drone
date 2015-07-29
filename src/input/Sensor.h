#ifndef SENSOR_VIRTUAL_H
#define SENSOR_VIRTUAL_H

#include "input/AxisTranslator.h"
/*
-Abstract base class for sensors that will be managed by an Inertial Manager
	Instance
-Inheritors of Sensor will
	A: handle all hardware communication code
	B: Convert outputs to standard units
	C: Do preliminary sensor filtering based on a sensor's native properties
	D: Pass data into corresponding interface of InerialManager when updated
*/

class Sensor{
protected:
public:
	enum Status {
		OK, BAD
	};
	virtual ~Sensor() {};
	virtual void   begin() = 0;
	virtual void   calibrate() = 0;
	virtual Status status() = 0;
	virtual void   end() = 0;
};

class InertialManager;
class InertialVec : public Sensor{
public:
	virtual void update(InertialManager& man, Translator axis) = 0;
};

#endif
