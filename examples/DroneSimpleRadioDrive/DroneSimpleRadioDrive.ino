#include <MINDS-i-Drone.h>

ServoGenerator::Servo drive, steer, backsteer;

void setup() {
  drive.attach(12 /* APM pin 1 */);
  steer.attach(11 /* APM pin 2 */);
  backsteer.attach(8 /* APM pin 3 */);

  ServoGenerator::begin();

  //set the initial throttle/direction for the ESC/servo
  drive.write(90);
  steer.write(90);
  backsteer.write(90);

  APMRadio::setup();
  Serial.begin(9600);

  //delay 2 seconds for arming
  delay(2000);
}

void loop() {
  static uint32_t timer = micros();
  uint32_t dt = timer+micros();
  timer = -micros();

  int driveSig = APMRadio::get(0);
  int steerSig = APMRadio::get(1);

  Serial.print(dt);
  Serial.print(" ");
  Serial.print(driveSig);
  Serial.print(" ");
  Serial.print(steerSig);
  Serial.println();

  drive.write( driveSig );
  steer.write( steerSig );
  backsteer.write( 90 - steerSig );
}
