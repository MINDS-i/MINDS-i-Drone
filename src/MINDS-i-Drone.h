#ifndef MINDSIDRONELIBS_H
#define MINDSIDRONELIBS_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

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

//#pragma GCC optimize ("Os")

//these macros can be turned off to save space
#define DEBUG 1
#if DEBUG
    #define TEST(a) Serial.print(#a);Serial.print(": ");Serial.print(a);Serial.print("\t");
    #define FAIL(a) {Serial.print("\nERROR:>"); Serial.println(a); return false; }
#else
    #define TEST(a) ;
    #define FAIL(a) return false;
#endif


#include "APM/APMRadioInput.h"
#include "APM/ServoGenerator.h"

#include "comms/CommManager.h"
#include "comms/NMEA.h"
#include "comms/Protocol.h"

#include "controllers/AltitudeHold.h"
#include "controllers/Horizon.h"
#include "controllers/PositionHold.h"

#include "filter/AcclOnly.h"
#include "filter/Altitude.h"
#include "filter/DualErrorFilter.h"
#include "filter/GyroOnly.h"
#include "filter/OrientationEngine.h"
#include "filter/RCFilter.h"
#include "filter/RCGyroFilter.h"

#include "input/altIMU/L3GD20H.h"
#include "input/altIMU/LPS25H.h"
#include "input/altIMU/LSM303D.h"
#include "input/altIMU/STMtwi.h"
#include "input/APM/HMC5883L.h"
#include "input/APM/LEA6H.h"

#include "input/APM/LEA6H_sim.h"

#include "input/APM/MPU6000.h"
#include "input/APM/MPU6000_HW.h"
#include "input/APM/MS5611.h"
#include "input/APM/Power.h"
#include "input/AxisTranslator.h"
#include "input/InertialManager.h"
//#include "input/Sensor.h"
#include "input/SPIcontroller.h"
#include "input/UM7.h"
#include "input/Bumper.h"


#include "math/Algebra.h"
#include "math/Quaternion.h"
#include "math/SpatialMath.h"
#include "math/Vec3.h"
#include "math/Waypoint.h"

#include "math/gps_angle.h"
#include "math/floatgps.h"
#include "math/gps_print.h"
#include "math/ftoa.h"

#include "output/AfroESC.h"
#include "output/EMaxESC.h"
#include "output/FlightStrategy.h"
#include "output/HK_ESCOutputDevice.h"
#include "output/OutputDevice.h"
#include "output/OutputManager.h"
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
#include "util/Interval.h"
#include "util/LTATune.h"
#include "util/PIDcontroller.h"
#include "util/PIDexternaltime.h"
#include "util/PIDparameters.h"
#include "util/profile.h"
#include "util/StateTimer.h"

#endif
