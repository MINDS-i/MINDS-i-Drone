#include "floatgps.h"
#include <ctype.h> // for toupper
#include <math.h>
#include <stdlib.h>
#include <string.h> // for strlen, strchr, strncpy

#include "ftoa.h"
#include "gps_angle.h"

/*****************************************************************************
FUNCTION: fearth_radius()
Calculate radius of earth in meters from GPS coordinates
6471000 mean radius

latitude is given in radians and is the geodedic latitude
Earth radius varies by +7000 m to -22000 m which is about
https://planetcalc.com/7721/
radius at equator (a) = 6378137 m
radius at pole (b) =6356752.3142 m
*****************************************************************************/
float fearth_radius(float latitude) {
    float radius_equator = 6379137.f;
    float radius_pole = 6356752.3142f;
    float rp_sin_lat = radius_pole * sinf(latitude);
    float re_cos_lat = radius_equator * cosf(latitude);
    float rp_sin_lat_2 = rp_sin_lat * rp_sin_lat;
    float re_cos_lat_2 = re_cos_lat * re_cos_lat;

    float denom = re_cos_lat_2 + rp_sin_lat_2;
    float numer = radius_equator * radius_equator * re_cos_lat_2 + radius_pole * radius_pole * rp_sin_lat_2;

    float radius = sqrtf(numer / denom);
    return radius;
} // end fearth_radius

/*****************************************************************************
FUNCTION: new_gps_float()
Calculate new  longitude using float precision
*****************************************************************************/
int new_gps_float(const GPS_COORD* gps_start, // [in] starting GPS coordinates
                  const LOCAL_COORD* local,   // [in] distance and heading
                  GPS_COORD* gps_end)         // [out] end GPS coordinates
{
    int err = 0;
    float lat_degrees = gps_angle_to_float(&gps_start->latitude);
    float long_degrees = gps_angle_to_float(&gps_start->longitude);
    // double dlat_degrees = gps_angle_to_double(&gps_start->latitude);
    // double dlong_degrees = gps_angle_to_double(&gps_start->longitude);

    // double phi1 = deg_to_rad(dlat_degrees);
    // double lambda1 = deg_to_rad(dlong_degrees);

    float fphi1 = fdeg_to_rad(lat_degrees);
    float flambda1 = fdeg_to_rad(long_degrees);

    float theta = fdeg_to_rad(local->heading);
    float delta = local->distance / fearth_radius(fphi1);

    // // for testing here...
    // float inc_sin_phi = -0.5f*delta*delta*sinf(fphi1)+delta*cosf(theta)*cosf(fphi1);
    // double sin_phi2 = sin(phi1) + inc_sin_phi;
    // double phi2 = asin(sin_phi2);
    // float delta_phi2 = (float)(phi2-phi1);

    // float inc_phi2_precise = delta * cosf(theta);  // this is faster, but not quite as precise
    float inc_phi2_precise = -0.5f * delta * delta * tanf(fphi1) + delta * cosf(theta);
    float inc_phi2_precise_deg = frad_to_deg(inc_phi2_precise);
    gps_angle_add_increment(&gps_start->latitude, inc_phi2_precise_deg, &gps_end->latitude);
    float fphi2_degrees = gps_angle_to_float(&gps_end->latitude);
    float fphi2 = fdeg_to_rad(fphi2_degrees);
    float y = sinf(theta) * delta * cosf(fphi1);
    float x = cosf(delta) - sinf(fphi1) * sinf(fphi2);
    float lambda_inc = atan2f(y, x);
    float lambda_inc_degrees = frad_to_deg(lambda_inc);
    gps_angle_add_increment(&gps_start->longitude, lambda_inc_degrees, &gps_end->longitude);

    return err;
} // end new_gps_float

/******************************************************************************
FUNCTION:  calc_delta_gps
determine the difference in latitude and longitude for two GPS points.
This assumes that the difference is small and can be represented as a
float.  This picks the shortest distance.
These differences are in range -180 < delta <= 180
******************************************************************************/
int calc_delta_gps(const GPS_COORD* gps1, // starting GPS
                   const GPS_COORD* gps2, // ending GPS
                   DELTA_GPS* delta_gps)  // difference in degrees (float)

{
    int err = 0;

    delta_gps->latitude = gps_angle_diff(&gps1->latitude, &gps2->latitude);
    delta_gps->longitude = gps_angle_diff(&gps1->longitude, &gps2->longitude);
    return err;
} // end calc_delta_gps

/******************************************************************************
FUNCTION: calc_haversine()
Calculate the distance and heading from two GPS coordinates using the Haversine
method
This uses the small angle approximation for delta phi and delta lambda
******************************************************************************/
int calc_haversine(const GPS_COORD* gps_start, // starting GPS (degrees)
                   const GPS_COORD* gps_end,   // ending GPS (degrees)
                   LOCAL_COORD* local)         // [out] local coordinates relative to start
{
    int err = 0;
    DELTA_GPS delta_gps;
    // converts two double gps to single float differences
    err = calc_delta_gps(gps_start, gps_end, &delta_gps);

    // if (delta_gps.latitude == 0.0 && delta_gps.longitude == 0.0) {
    //     local->distance = 0.0;
    //     local->heading = 0.0;
    //     local->x = 0.0;
    //     local->y = 0.0;
    //     return err;
    // }

    float delta_phi = fdeg_to_rad(delta_gps.latitude);
    float delta_lambda = fdeg_to_rad(delta_gps.longitude);
    float fphi1_deg = gps_angle_to_float(&gps_start->latitude);
    float fphi1 = fdeg_to_rad(fphi1_deg);
    float flambda1_deg = gps_angle_to_float(&gps_start->longitude);
    float flambda1 = fdeg_to_rad(flambda1_deg);
    float fphi2_deg = gps_angle_to_float(&gps_end->latitude);
    float fphi2 = fdeg_to_rad(fphi2_deg);
    float flambda2_deg = gps_angle_to_float(&gps_end->longitude);
    float flambda2 = fdeg_to_rad(flambda2_deg);

    // float sin_delta_phi = sinf(delta_phi/2.f);
    float sin_delta_phi = delta_phi / 2.f; // small angle sinf approximation
    sin_delta_phi *= sin_delta_phi;
    // float sin_delta_lambda = sinf(delta_lambda/2.f);
    float sin_delta_lambda = delta_lambda / 2.f;
    sin_delta_lambda *= sin_delta_lambda;
    float a = sin_delta_phi + cosf(fphi1) * cos(fphi2) * sin_delta_lambda;
    // float c = 2.f * atan2f(sqrtf(a), sqrtf(1-a));
    float c = 2.f * sqrtf(a); // small angle approx
    local->distance = c * fearth_radius(fphi1);

    // float y = sinf(delta_lambda) * cosf(fphi2);
    // float x = cosf(fphi1) * sinf(fphi2) - sinf(fphi1) * cosf(fphi2) * cosf(delta_phi);
    float y = delta_lambda * cosf(fphi2); // small angle approx
    // float x = cosf(fphi1) * sinf(fphi2) - sinf(fphi1) * cosf(fphi2); // small angle approx
    // float x = sin(fphi2 - fphi1);  // another small angle approx
    float x = delta_phi;        // yet another small angle approx
    float theta = atan2f(y, x); // theta is not necessarily a small angle so keep this
    float theta_deg = frad_to_deg(theta);
    theta_deg += 360.f;
    if (theta_deg >= 360.f) {
        theta_deg -= 360.f;
    }

    local->heading = theta_deg;
    local->x = x;
    local->y = y;

    return err;
} // end calc_haversine

/******************************************************************************
FUNCTION gps_coord_to_str()
Convert both latitude and longitude to strings
******************************************************************************/
int gps_coord_to_str(GPS_COORD* gps,  // coordinates to convert
                     char* str,       // place to put string
                     int len,         // length of string
                     int num_dec,     // number of digits after decimal place
                     const char* fmt) // one of "DD", "DMM", or "DMS"
{
    int err = 0;
    int elat = 0;
    int elon = 0;
    elat = lonlat_to_str(&gps->latitude, str, len, num_dec, fmt);
    if (!elat) {
        strcat(str, ",");
        int lat_len = strlen(str);
        elon = lonlat_to_str(&gps->longitude, str + lat_len, len - lat_len, num_dec, fmt);
    }
    if (elon || elat) {
        err = GPS_ANGLE_ERR_BAD_CONVERSION;
    }
    return err;
} // end gps_coord_to_str
