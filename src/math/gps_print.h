/******************************************************************************
FILE: gps_print.h
auxilliarly files to print things
******************************************************************************/
#ifndef GPS_PRINT_H
#define GPS_PRINT_H

#include "Arduino.h"

#include "floatgps.h"
#include "gps_angle.h"

// some debugging functions
void print_gps(HardwareSerial *outputSerial, char* title, GPS_COORD* gps);
void print_local(HardwareSerial *outputSerial, char* title, LOCAL_COORD* local);
void print_delta(HardwareSerial *outputSerial, char* title, DELTA_GPS* delta_gps);

void print_gps_angle(HardwareSerial *outputSerial, char* title, GPS_ANGLE* angle); // debug print of GPS_ANGLE

#endif

