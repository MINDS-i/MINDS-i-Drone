/*
Copyright 2024 MINDS-i Inc.

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

#ifndef MINDS_I_DRONE_COMMON_PING_H
#define MINDS_I_DRONE_COMMON_PING_H

#include <util/atomic.h>
#include "Arduino.h"
#include "wiring_private.h"

/** maximum microseconds before considering a ping sensor reading lost */
constexpr uint16_t PING_READING_TIMEOUT = 20000;
/** maximum loop count for a QTI sensor reading */
constexpr uint16_t QTI_READING_TIMEOUT = 10000;

/** Activate a parallax ping sensor and return the echo time in microseconds */
uint16_t getPing(int pin, uint16_t maxMicros = PING_READING_TIMEOUT);
/** Poll a QTI sensor, returning a unitless time based value
  * smaller values correspond to a higher light intensity hitting the sensor
  */
uint16_t QTI(int pin, uint16_t maxLoops = QTI_READING_TIMEOUT);

#endif // MINDS_I_DRONE_COMMON_PING_H
