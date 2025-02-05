#include "MINDS-i-Drone.h"

bumper bumperSensor;

void setup() {

    Serial.begin(115200);
    delay(500);

    bumperSensor.begin(A5, A6);
    Serial.println("Starting Bumper sensor test");
}

void loop() {

    bumperSensor.update();

    if (bumperSensor.leftButtonEvent()) {
        if (bumperSensor.leftButtonState()) {
            Serial.println("Left Button Pressed");
        } else {
            Serial.println("Left Button Released");
        }
    }

    if (bumperSensor.rightButtonEvent()) {
        if (bumperSensor.rightButtonState()) {
            Serial.println("Right Button Pressed");
        } else {
            Serial.println("Right Button Released");
        }
    }

    delay(25);
}