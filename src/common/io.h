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

#ifndef MINDS_I_DRONE_COMMON_IO_H
#define MINDS_I_DRONE_COMMON_IO_H

#include <util/atomic.h>
#include "Arduino.h"
#include "wiring_private.h"

/** read a digital pin quicker than normal digitalRead
     * by leaving out checks for the pin existing and its PWM mode
     */
inline bool fastDigitalRead(int pin){
    return *portInputRegister(digitalPinToPort(pin))
            & digitalPinToBitMask(pin);
}

#endif // MINDS_I_DRONE_COMMON_IO_H
