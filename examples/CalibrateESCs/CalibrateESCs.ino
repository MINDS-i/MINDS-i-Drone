#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
#include "platforms/Quadcopter.h"

void setup(){
    calibrateESCs();
}
void loop(){
    output.stop();
}
