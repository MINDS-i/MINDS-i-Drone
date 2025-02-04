/* Copyright 2015 MINDS-i, INC.

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

#include "Ping.h"

uint16_t getPing(int pin, uint16_t maxMicros) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(5);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);
    delayMicroseconds(5);
    pinMode(pin, INPUT);
    int inputpulse = pulseIn(pin, HIGH, maxMicros);
    if (inputpulse == 0)
        inputpulse = maxMicros;
    delayMicroseconds(200);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    return inputpulse;
}

uint16_t QTI(int pin, uint16_t maxLoops) {
    uint16_t time = 0;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delay(1);
    pinMode(pin, INPUT);
    digitalWrite(pin, LOW);
    while (digitalRead(pin)) {
        time++;
        if (time >= maxLoops)
            break;
    }
    return time;
}
