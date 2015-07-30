#ifndef SENSOR_VIRTUAL_H
#define SENSOR_VIRTUAL_H

#include "input/AxisTranslator.h"
/*
-Inheritors of Sensor will
	A: handle all hardware communication code
	B: Convert outputs to standard units
	C: Do preliminary sensor filtering based on a sensor's native properties
	D: Keep track of the sensor's status
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
/*
Inertial Vector sensors are sensors that  read a 3-vector relating
	to the sensor's orientation. They provide an extra method for the
	inertial manager to call on them, in which they will use the translator
	to transform from the sensor frame to NED and reference passed in to
	change the inertial manager's state according to their reading.

	The inertial manager class will need to friend any InertialVecs before
	they can alter its state
*/
class InertialManager;
class InertialVec : public Sensor{
public:
	virtual void update(InertialManager& man, Translator axis) = 0;
};

#endif
