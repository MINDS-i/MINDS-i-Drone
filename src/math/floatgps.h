/*****************************************************************************
FILE: floatgps.h
This defines structures and functions for handling GPS coordinates using
floats where possible instead of double
*****************************************************************************/
#if !defined (FLOATGPS_H)
#define FLOATGPS_H

#include "gps_angle.h"


typedef struct
{
  GPS_ANGLE latitude;  // latitude (degrees)
  GPS_ANGLE longitude; // longitude (degrees)
} GPS_COORD;

typedef struct
{
  float distance;     // distance meters
  float heading;  // degrees east on north
} LOCAL_COORD;

typedef struct
{
  float latitude;  // degrees difference in latitude
  float longitude; // degress difference in longitude
} DELTA_GPS;


float fearth_radius(float latitude);



// int new_gps_exact(GPS_COORD* gps_start, // [in] starting GPS coordinates
//                   LOCAL_COORD* local, // [in] distance and heading
//                   GPS_COORD* gps_end); // [out] end GPS coordinates

int new_gps_float(GPS_COORD* gps_start, // [in] starting GPS coordinates
                  LOCAL_COORD* local,  // [in] distance and heading
                  GPS_COORD* gps_end); // [out] end GPS coordinates

int calc_delta_gps(GPS_COORD* gps1,  // [in] starting GPS
              GPS_COORD* gps2,       // [in] ending GPS
              DELTA_GPS* delta_gps); // [out] difference in degrees (float)

int calc_haversine(GPS_COORD* gps_start, // [in] starting GPS (degrees)
                   GPS_COORD* gps_end,   // [in] ending GPS (degrees)
                   LOCAL_COORD* local);  // [out] local coordinates relative to start
int gps_coord_to_str(GPS_COORD* gps,  // coordinates to convert
                     char* str, // place to put string
                     int len,   // length of string
                     int num_dec, // number of digits after decimal place
                     char* fmt);  // one of "DD", "DMM", or "DMS"

#endif

