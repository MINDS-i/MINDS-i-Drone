/******************************************************************************
FILE: gps_angle.c
Handles high precision angles using float
******************************************************************************/

#include <math.h>
#include <string.h> // for strlen, strchr, strncpy
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>  // for toupper
//#include "global_defs.h"  // DEBUG_PRINT and DEBUG_DOUBLE
#include "floatgps.h"
#include "ftoa.h"


#define MIN_IN_180 10800
#define MIN_IN_360 21600

/******************************************************************************
FUNCTION:  gps_angle_to_float()
Converts GPS_ANGLE value to float degrees
Loses some precision
Used to calculate sin and cos when precision is not critical
******************************************************************************/
float gps_angle_to_float(GPS_ANGLE* angle)
{
    float result;
    result = (float)angle->minutes + angle->frac;
    result /= 60.f;
    return result;
} // end gps_angle_to_float

/******************************************************************************
FUNCTION:  float_to_gps_angle()
convert a float degrees to GPS_ANGLE
******************************************************************************/
void float_to_gps_angle(float degrees, GPS_ANGLE* gps)
{
    float dminutes = degrees * 60.;
    double frac = fmod(dminutes, 1.);
#if DEBUG_PRINT
    printf("float_to_gps_angle: degrees = %9e\n", degrees);
    printf("float_to_gps_angle: dminutes = %.9e\n", dminutes);
    printf("float_to_gps_angle: frac = %.9e\n", frac);
#endif   
    gps->minutes = (int)(dminutes - frac);
    gps->frac = (float)frac;
#if DEBUG_PRINT
    print_gps_angle("float_to_gps_angle", gps);
#endif
    gps_angle_normalize(gps);
#if DEBUG_PRINT
    print_gps_angle("after normalize", gps);
#endif
} //end float_to_gps_angle

/******************************************************************************
FUNCTION:  gps_angle_normalize()
checks the minutes and frac parts of the angle and makes the fractional part
be between -1.0 < frac < 1.0
For LONGITUDE, makes the angle -180 < angle <= 180.
******************************************************************************/
void gps_angle_normalize(GPS_ANGLE* angle)
{
    int minutes = angle->minutes;
    float frac = angle->frac;
    float r = fmod(frac, 1.f);  // fractional part with sign
    float i = frac - r;    // integral part with sign
    minutes += (int)i;
    frac = r;
    // if signs are different, then adjust
    if (frac != 0)
    {
        if (frac > 0.f && minutes < 0.f)
        {
             minutes += 1.f;
             frac -= 1.f;
        }
        else if (frac < 0.f && minutes > 0.f)
        {
             minutes -= 1.f;
             frac += 1.f;
        }
    }
    angle->minutes = minutes;
    angle->frac = frac;
    if (angle->type == LONGITUDE_TYPE)
    {
        if (angle->minutes > MIN_IN_180)
        {
            angle->minutes -= MIN_IN_360;
        }
        else if (angle->minutes <= -MIN_IN_180)
        {
            angle->minutes += MIN_IN_360;
        }
    }
} // end gps_angle_normalize

/******************************************************************************
FUNCTION:  gps_angle_negate()
change sign of angle
******************************************************************************/
void gps_angle_negate(GPS_ANGLE* angle)
{
    angle->minutes = -angle->minutes;
    angle->frac = -angle->frac;
}// end gps_angle_negate

/******************************************************************************
FUNCTION:  gps_angle_apply_sign()
change sign of angle
******************************************************************************/
void gps_angle_apply_sign(GPS_ANGLE* angle, char dir)
{
    char c = toupper(dir);
    if (angle->type == LATITUDE_TYPE)
    {
        if (c == 'S')
        {
            gps_angle_negate(angle);
        }
    }
    else if (angle->type == LONGITUDE_TYPE)
    {
        if (c == 'W')
        {
            gps_angle_negate(angle);
        }
    }
} //end gps_angle_apply_sign

/******************************************************************************
FUNCTION:  gps_angle_diff()
subtracts two GPS_ANGLES and returns difference as float degrees
latitude is always monotonic, so a simple subtraction will always work.
we pick the shortest distance and know that the range is
-180 < longitude <= 180
if the difference is greater than 180 degrees, then go the other way.
180 * 60 = 1080 minutes in 180 degrees
2160 is number of minutes in 360 degrees
This uses the type of the first angle.  Assumes both are the same
******************************************************************************/
float gps_angle_diff(GPS_ANGLE* gps1, GPS_ANGLE* gps2)
{
    float result;
    GPS_ANGLE diff;
    diff.type = gps1->type;
    diff.minutes = gps2->minutes - gps1->minutes;
    diff.frac = gps2->frac - gps1->frac;
    gps_angle_normalize(&diff);
    result = (float)diff.minutes + diff.frac;
    result /= 60.f; // convert to degrees
    return result;
} // end gps_angle_diff

/******************************************************************************
FUNCTION:  gps_angle_add_increment
Adds a float in degrees to GPS_ANGLE
returns error if |lat| >= 90
******************************************************************************/
int gps_angle_add_increment(GPS_ANGLE* gps,      // [in] gps coordinate
                              float diff,         // [in] small increment in degrees
                              GPS_ANGLE* result)  // [out] gps + increment
{
    int err = 0;
    *result = *gps;  // copy input to output including type
    float diff_minutes = diff * 60;  // convert to minutes
    result->frac = gps->frac + diff_minutes;
    gps_angle_normalize(result);
    return err;    
} // end gps_angle_add_increment

/*****************************************************************************
FUNCTION: get_lonlat()
convert string to gps_angle
string has DDDMM.MMMMMM
DD or DDDis degrees , MM.MMMMM is minutes
Convert ot DDDMM minutes and .MMMMMM for fractional part
*****************************************************************************/
int get_lonlat(char* str, GPS_ANGLE* lonlat)
{
    int err = 0;
    char buf[20];
    char* end_ptr;
    size_t len = strlen(str);
    if (len >= sizeof(buf)-1)
    {
        return -1;
    }
    strcpy(buf, str);
    char* p = strchr(buf, '.');
    int first_part;
    float second_part;
    
    if (p == NULL)
    {
        first_part = strtod(buf, &end_ptr);
        second_part = 0.f;
    }
    else
    {
        *p = 0;  // replace . with 0
        first_part = atoi(buf);  // convert as integer
        *p = '.';  // replace .
        second_part = strtod(p, &end_ptr);
    }
    int degrees = first_part / 100;
    int minutes = first_part - degrees * 100;
    lonlat->minutes = minutes + degrees * 60;
    lonlat->frac = second_part;
    gps_angle_normalize(lonlat);
    return err;
        
}// end get_lonlat


/*****************************************************************************
FUNCTION: get_longitude()
convert longitude string to gps_angle
string has DDDMM.MMMMMM
DD is degrees longitude, MM.MMMMM is minutes
Convert ot DDDMM minutes and .MMMMMM for fractional part
set LONGITUDE_TYP
*****************************************************************************/
int get_longitude(char* str, GPS_ANGLE* longitude)
{
    int err = 0;
    longitude->type = LONGITUDE_TYPE;
    err = get_lonlat(str, longitude);
    return err;
} // end get longitude

/*****************************************************************************
FUNCTION: get_latitude()
convert latitude string to gps_angle
string has DDMM.MMMMMM
DD is degrees latitude, MM.MMMMM is minutes
Convert ot DDMM minutes and .MMMMMM for fractional part
set LATITUDE_TYPE
*****************************************************************************/
int get_latitude(char* str, GPS_ANGLE* latitude)
{
    int err = 0;
    latitude->type = LATITUDE_TYPE;
    err = get_lonlat(str, latitude);
    return err;
} //end get_latitude

/******************************************************************************
FUNCTION: lonlat_to_str()
convert longitutude or latitude from GPS_ANGLE to string
latitude: DDMM.MMMMM N/S
longitude: DDDMM.MMMMM E/W

return 0 if ok
possible errors,
GPS_ANGLE_ERR_TOO_LONG - buffer not long enough
GPS_ANGLE_ERR_RANGE - input angle not in proper range
DD = decimal degrees DD.DDDDDDDD (signed)
DMM = decimal degrees and decimal minutes DD MM.MMMMMMMM (space between, DD has sign)
DMS = degrees, minutes, and seconds direction, space betwen lat and long
******************************************************************************/
int lonlat_to_str(GPS_ANGLE* lonlat, // [in] angle to convert
                  char* str, // [in] buffer to place string
                  int len,  // [in] length of buffer including null
                  int num_dec, // [in] number of digits after decimal place
                  char* fmt)   // [in] one of "DD", "DMM", or "DMS"
{
    int err = 0;
    if (lonlat->type != LONGITUDE_TYPE && lonlat->type != LATITUDE_TYPE)
    {
        return GPS_ANGLE_ERR_BAD_TYPE;
    }
    char dir;
    char delim = ',';
    int neg = 0;
    float abs_frac = lonlat->frac;
    int abs_minutes = lonlat->minutes;
    int deg_digits;
    if (abs_minutes < 0)
    {
        abs_minutes = -abs_minutes;
        abs_frac = -abs_frac;
        neg = 1;
    }
    int degrees = abs_minutes / 60; // truncate
    abs_minutes -= (degrees * 60);
    float angle = gps_angle_to_float(lonlat);
    if (lonlat->type == LONGITUDE_TYPE)
    {
        // DDDMM.MMMMM,N/M<NULL>
        int max_len = num_dec + 9;
        if (len < max_len)
        {
            return GPS_ANGLE_ERR_TOO_LONG;
        }
        if (angle >180.f || angle <= -180.f)
        {
            return GPS_ANGLE_ERR_BAD_RANGE;
        }
        if (neg)
        {
            dir = 'W';
        }
        else
        {
            dir = 'E';
        }
        deg_digits = 3;
    }
    else  // must be LATITUDE_TYPE
    {
        // DDMM.MMMMM,N/S<NULL>
        int max_len = num_dec + 8;
        if (len < max_len)
        {
            return GPS_ANGLE_ERR_TOO_LONG;
        }
        if (angle > 90.f || angle < -90.f)
        {
            return GPS_ANGLE_ERR_BAD_RANGE;
        }
        if (neg)
        {
            dir = 'S';
        }
        else
        {
            dir = 'N';
        }
        deg_digits = 2;
    }
    // angles are in range and the buffer is big enough.
    // create a format string
    if (strcmp(fmt, "DMM") == 0)
    {
        char fmt_deg[10];
        char* ptr = str;
        ptr += sprintf(ptr, "%d", degrees);  // don't print leading zero
        ptr += sprintf(ptr, "%02d", abs_minutes);
        err = ftoa_frac(abs_frac, ptr, len - (ptr-str), num_dec);
        if (ptr - str > len)
        {
            err = GPS_ANGLE_ERR_TOO_LONG;
        }
    } // end if "DMM"
    else if (strcmp(fmt, "DMS") == 0)
    {
        char* ptr = str;
        ptr += sprintf(ptr, "DMS");
    }// end if DMS
    else if (strcmp(fmt, "DD") == 0)
    {
        char* ptr = str;
        if (neg)
        {
            ptr += sprintf(ptr, "-");
        }
        ptr += sprintf(ptr, "%d", degrees);
        err = ftoa_minutes_to_deg_str(abs_minutes, abs_frac, ptr, len - (ptr-str));
#if DEBUG_PRINT
        if (err != 0)
        {
            printf("ftoa_minutes_to_deg_str return err = %d\n", err);
        }
#endif
    } // end if "DD"
    return err;
} // end lonlat_to_str

/******************************************************************************
FUNCTION: fdeg_to_rad()
Convert gps coordinates in degrees to normalized radians using float

******************************************************************************/
float fdeg_to_rad(float degrees)
{
    float radians = (float)M_PI * degrees / 180.f;
    return radians;
}

/******************************************************************************
FUNCTION: frad_to_deg()
Convert radians to degrees with float type

******************************************************************************/
float frad_to_deg(float radians)
{
    float deg = 180.f* radians / M_PI;
    return deg;
}

