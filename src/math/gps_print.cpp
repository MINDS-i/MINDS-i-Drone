/******************************************************************************
FiLE: gps_print.c
Functions for debug printing
******************************************************************************/

#include "gps_print.h"
#include "floatgps.h"
#include "gps_angle.h"

// some debugging functions
void print_gps(HardwareSerial* outputSerial, const char* title, GPS_COORD* gps) {
    outputSerial->print("GPS: ");
    outputSerial->println(title);

    outputSerial->print("latitude: Minutes = ");
    outputSerial->print(gps->latitude.minutes);
    outputSerial->print(", frac = ");
    outputSerial->print(gps->latitude.frac, 9);
    outputSerial->print(", Type = ");
    outputSerial->println(gps->latitude.type, 9);

    outputSerial->print("longitude: Minutes = ");
    outputSerial->print(gps->longitude.minutes);
    outputSerial->print(", frac = ");
    outputSerial->print(gps->longitude.frac, 9);
    outputSerial->print(", Type = ");
    outputSerial->println(gps->longitude.type, 9);

    // printf("  latitude: Minutes = %d, frac = %.9e\n",
    //         gps->latitude.minutes, gps->latitude.frac);
    // printf("  longitude: Minutes = %d, frac = %.9e\n",
    //         gps->longitude.minutes, gps->longitude.frac);
    // printf("latitude type = %d, longitude type = %d\n",
    //         gps->latitude.type, gps->longitude.type);
}

void print_local(HardwareSerial* outputSerial, char* title, LOCAL_COORD* local) {
    outputSerial->print("LOCAL: ");
    outputSerial->println(title);
    outputSerial->print("  distance = ");
    outputSerial->print(local->distance, 9);
    outputSerial->println("m");
    outputSerial->print("  heading = ");
    outputSerial->print(local->heading, 9);
    outputSerial->println("degrees");

    // printf("\nLOCAL: %s\n", title);
    // printf("  distance = %.9e m\n", local->distance);
    // printf("  heading = %.9e degrees\n", local->heading);
}

void print_delta(HardwareSerial* outputSerial, char* title, DELTA_GPS* delta_gps) {
    outputSerial->print("DELTA_GPS: ");
    outputSerial->println(title);
    outputSerial->print("  delta lat = ");
    outputSerial->print(delta_gps->latitude, 9);
    outputSerial->println("degrees");
    outputSerial->print("  delta long = ");
    outputSerial->print(delta_gps->longitude, 9);
    outputSerial->println("degrees");

    // printf("\nDELTA GPS: %s\n", title);
    // printf(" delta lat = %.9e degrees\n", delta_gps->latitude);
    // printf(" delta long = %.9e degrees\n", delta_gps->longitude);
}

/******************************************************************************
FUNCTION: print_gps_angle()
debug print routine for GPS_ANGLE
Displays title on first line
Displays values following
******************************************************************************/
void print_gps_angle(HardwareSerial* outputSerial, char* title, GPS_ANGLE* angle) {
    outputSerial->print("GPS_ANGLE: ");
    outputSerial->println(title);
    outputSerial->print("  type = ");
    outputSerial->println(angle->type);
    outputSerial->print("  minutes = ");
    outputSerial->println(angle->minutes);
    outputSerial->print("  frac = ");
    outputSerial->println(angle->frac, 9);

    // printf("\nGPS_ANGLE: %s\n", title);
    // printf(" type = %d, minutes = %d, frac = %.9e\n",
    //        angle->type, angle->minutes, angle->frac);
}
