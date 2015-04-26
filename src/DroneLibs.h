#ifndef DRONELIBS_H
#define DRONELIBS_H

#include "Arduino.h"
/*
Copyright 2015 MINDS-i Inc.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

//take this macro; it might come in handy
#define TEST(a) Serial.print(#a);Serial.print(": ");Serial.print(a);Serial.print("\t");

#include "APM/MegaInterrupts.h"
#include "APM/APMRadioInput.h"

#include "comms/CommManager.h"
#include "comms/NMEA.h"
#include "comms/Protocol.h"

#include "filter/AcclOnly.h"
#include "filter/DualErrorFilter.h"
#include "filter/DualErrorParams.h"
#include "filter/GyroOnly.h"
#include "filter/OrientationEngine.h"

#include "input/altIMU/L3GD20H.h"
#include "input/altIMU/LPS25H.h"
#include "input/altIMU/LSM303D.h"
#include "input/altIMU/STMtwi.h"
#include "input/APM/LEA6H.h"
#include "input/APM/MS5611.h"
#include "input/APM/MPU6000.h"
#include "input/APM/HMC5883L.h"
#include "input/InertialManager.h"
#include "input/Sensor.h"
#include "input/SPIcontroller.h"

#include "math/GreatCircle.h"
#include "math/Quaternion.h"
#include "math/SpatialMath.h"
#include "math/Vec3.h"
#include "math/Waypoint.h"
#include "math/Algebra.h"

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
