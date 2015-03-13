#ifndef DRONELIBS_H
#define DRONELIBS_H

#include "Arduino.h"

//take this macro; it might come in handy
#define TEST(a) Serial.print(#a);Serial.print(": ");Serial.print(a);Serial.print("\t");

#include "APM/Compass.h"
#include "APM/MegaInterrupts.h"
#include "APM/MPU.h"

#include "comms/CommManager.h"
#include "comms/NMEA.h"
#include "comms/Protocol.h"

#include "filter/DualErrorFilter.h"
#include "filter/DualErrorParams.h"
#include "filter/OrientationEngine.h"

#include "input/altIMU/L3GD20H.h"
#include "input/altIMU/LPS25H.h"
#include "input/altIMU/LSM303D.h"
#include "input/altIMU/STMtwi.h"
#include "input/APMCompass.h"
#include "input/APMRadioInput.h"
#include "input/InertialManager.h"
#include "input/InertialSensor.h"
#include "input/MpuSensor.h"
#include "input/MS5611.h"

#include "math/GreatCircle.h"
#include "math/Quaternion.h"
#include "math/SpatialMath.h"
#include "math/Vec3.h"
#include "math/Waypoint.h"

#include "output/HK_ESCOutputDevice.h"
#include "output/OutputDevice.h"
#include "output/OutputManager.h"
#include "output/OutputSolverCross.h"
#include "output/ServoOutputDevice.h"

#include "storage/EEPROMlist.h"
#include "storage/EEPROMstorage.h"
#include "storage/EEPROMsubsystem.h"
#include "storage/List.h"
#include "storage/queue.h"
#include "storage/Settings.h"
#include "storage/SRAMlist.h"
#include "storage/SRAMstorage.h"
#include "storage/Storage.h"

#include "util/byteConv.h"
#include "util/callbackTemplate.h"
#include "util/HLAverage.h"
#include "util/LTATune.h"
#include "util/PIDcontroller.h"
#include "util/PIDparameters.h"
#include "util/profile.h"
#include "util/UM7.h"

#endif
