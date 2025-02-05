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

#ifndef MINDS_I_DRONE_COMMON_ENCODER_H
#define MINDS_I_DRONE_COMMON_ENCODER_H

#include "io.h"

class encoder {
  private:
    static const uint16_t timestep = 10;
    static const uint8_t sampleSize = 8;
    // static const float TpR = 100.0f; //ticks per revolution
    static const float TpR; // ticks per revolution
    static const float rpmConvFactor;
    static int16_t average[sampleSize];
    static int16_t* avptr;
    static uint32_t time;
    static int8_t pin[2];
    static int16_t tickCount, lastCount;
    static void stepAverage() {
        while (millis() - time > timestep) {
            time += timestep;
            pushAverage(tickCount - lastCount);
            lastCount = tickCount;
        }
    }
    static void interrupt(void) {
        stepAverage();
        if (pin[1] != -1 && fastDigitalRead(pin[1])) {
            return;
        }
        tickCount++;
    }
    static void Binterrupt(void) {
        if (fastDigitalRead(pin[0])) {
            return;
        }
        tickCount--;
    }
    static void pushAverage(int16_t val) {
        *avptr = val;
        avptr++;
        if (avptr >= &average[sampleSize]) {
            avptr = average;
        }
    }

  public:
    static void begin(uint8_t pinA) {
        if (digitalPinToInterrupt(pinA) == NOT_AN_INTERRUPT) {
            return;
        }

        // check if the interrupt is already on

        pin[0] = pinA;
        pin[1] = -1;

        uint8_t oldSREG = SREG;
        cli();
        attachInterrupt(digitalPinToInterrupt(pin[0]), &encoder::interrupt, RISING);
        SREG = oldSREG;
    }
    static void begin(uint8_t pinA, uint8_t pinB) {
        if (digitalPinToInterrupt(pinA) == NOT_AN_INTERRUPT) {
            return;
        }
        if (digitalPinToInterrupt(pinB) == NOT_AN_INTERRUPT) {
            return;
        }

        // check if the interrupt is already on

        pin[0] = pinA;
        pin[1] = pinB;

        uint8_t oldSREG = SREG;
        cli();
        attachInterrupt(digitalPinToInterrupt(pin[0]), &encoder::interrupt, RISING);
        attachInterrupt(digitalPinToInterrupt(pin[1]), &encoder::Binterrupt, RISING);
        SREG = oldSREG;
    }
    static float getRPM() {
        stepAverage();
        int32_t avsum = 0;
        for (int i = 0; i < sampleSize; i++) {
            avsum += average[i];
        }
        return ((float)avsum) * rpmConvFactor;
    }
    static float getRev() { return float(tickCount) / TpR; }
};
// static var initializations
int16_t encoder::average[] = {0};
int8_t encoder::pin[] = {-1};
int16_t encoder::tickCount = 0;
int16_t encoder::lastCount = 0;
uint32_t encoder::time = 0;
int16_t* encoder::avptr = average;
const float encoder::TpR = 100.0f; // ticks per revolution
const float encoder::rpmConvFactor =
    // ms per min  / (samples per avsum)*(ms per sample)*(ticks per rev)
    (60.f * 1000.f) / ((encoder::sampleSize) * (encoder::timestep)*100.0f);

#endif // MINDS_I_DRONE_COMMON_ENCODER_H
