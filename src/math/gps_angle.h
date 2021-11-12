/*****************************************************************************
FILE: gps_angle.h
This defines structures and functions for handling precision angles with
floats
*****************************************************************************/
#if !defined (GPS_ANGLE_H)
#define GPS_ANGLE_H

#include <stdlib.h>
#define LATITUDE_TYPE 0   // -90 < lat < 90, error if not in range
#define LONGITUDE_TYPE 1  // -180 < long <= 180, modulo 360
#define GPS_ANGLE_ERR_TOO_LONG (-1)
#define GPS_ANGLE_ERR_BAD_RANGE (-2)
#define GPS_ANGLE_ERR_BAD_TYPE (-3)
#define GPS_ANGLE_ERR_BAD_CONVERSION (-4)
typedef struct
{
    int minutes;    // minutes of arc
    float frac;  // fraction of minute of arc
    char type;   //  LATITUDE_TYPE or LONGITUDE_TYPE
} GPS_ANGLE;

int get_lonlat(char* str, GPS_ANGLE* lonlat); // convert string to GPS_ANGLE 
int lonlat_to_str(const GPS_ANGLE* lonlat, // [in] angle to convert
                  char* str, // [in] buffer to place string
                  int len,  // [in] length of buffer including null
                  int num_dec, // [in] number of digits after decimal place
                  const char* fmt);   // [in] one of "DD", "DMM", or "DMS"
int get_longitude(char* str, GPS_ANGLE* longitude); // convert string longitude to GPS_ANGLE
int get_latitude(char* str, GPS_ANGLE* latitude); // convert string latitude to GPS_ANGLE

float gps_angle_to_float(const GPS_ANGLE* angle); // angle in degrees, loses some precision
void float_to_gps_angle(float degrees, GPS_ANGLE* gps); // loses precision
float gps_angle_diff(const GPS_ANGLE* gps1, const GPS_ANGLE* gps2);  // difference in degrees (gps2 - gps1)
int gps_angle_add_increment(const GPS_ANGLE* gps, // [in] input gps coordinate
                              float inc, // [in] small increment in degrees
                              GPS_ANGLE* result);  // [out] gps + increment
void gps_angle_normalize(GPS_ANGLE* angle);  // adjust frac and minutes so -1.f < frac < 1.f
void gps_angle_negate(GPS_ANGLE* angle);  // change sign of GPS_ANGLE
void gps_angle_apply_sign(GPS_ANGLE* angle, char dir); // apply E|W to longitude, N|S to latitude

// rad and degree conversions
float fdeg_to_rad(float degrees);
float frad_to_deg(float radians);

#endif



