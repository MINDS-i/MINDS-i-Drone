/******************************************************************************
FILE: ftoa.c
functions to help with converting float to ascii string
******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include "ftoa.h"

#define DEBUG_FTOA_MINUTES 0

/******************************************************************************
FUNCTION: ftoa_frac()
Convert float to ascii string

Multiply by 10, assign to unsigned int store in digit
subtract result
do until number of digits is determined
The remainder at the end allows rounding
Rounding will be done on the string.

This assumes a positive or zero float between 0 <= x < 1.0
result will be ".ddddddd" with precision digits

******************************************************************************/
int ftoa_frac(float x,  //[in] numberto convert
         char* buf,     // [in] place to put ASCII
         int len,       // [in] length of buffer including null
         int precision) // [in] number of digits after decimal
{
    int err = 0;
    float y = x;
    int n;
    if (len < precision + 2)  // length plus decimal and null
    {
        err = FTOA_ERR_TOO_LONG;
        return err;
    }
    if (x < 0.f || x >= 1.0f)
    {
        err = FTOA_ERR_BAD_VALUE;
    }
    char* ptr = buf;
    *ptr++ = '.';
    for(n = 0; n < precision; n++)
    {
        unsigned int temp;
        y *= 10.f;
        temp = (unsigned int) y;
#if DEBUG_FTOA_MINUTES
        printf("temp = %d\n", temp);
#endif
        *ptr++ = '0' + temp;
        y -= (float)temp;
    }
    *ptr++ = '\0';
#if DEBUG_FTOA_MINUTES
    printf("ftoa_frac buf = %s\n", buf);
#endif
    if (y >= 0.5f)
    {
        err = ftoa_roundup(buf, precision);
#if DEBUG_FTOA_MINUTES
        if (err != 0)
        {
            printf("ftoa_roundup returned %d\n", err);
        }
        else
        {
            printf("rounded up to %s\n", buf);
        }
#endif
    }

    return err;

} // end ftoa_frac

/******************************************************************************
FUNCTION: ftoa_roundup()
roundup the ASCIi string by adding 1 and carrying through the digits
input is a string starting with .
len is index of first digit to start.  Work backwords until no carry or .
or beginning of buffer
Error if carry is beyond the first character
In this case the value rounds to 1.0
The calling function should check for this and needs to add 1 to integerpart

******************************************************************************/
int ftoa_roundup(char* buf,        // [in,out] pointer to buffer with number to round
                 int start_index)  // [in] index to start with

{
    int err = 0;
    int i;
    int carry = 0;
    int end_i = 1;
    if (buf[0] != '.')
    {
        end_i = 0;
    }
    for (i = start_index; i >= end_i; i--)
    {
        char digit = buf[i];
        if (digit < '0' || digit > '9')
        {
            err = FTOA_ERR_BAD_CHAR;
            return err;
        }
        if (digit == '9')
        {
             buf[i] = '0';
             carry = 1;
        }
        else
        {
            buf[i] += 1;
            carry = 0;
            break;  // done
        }
    } // end for(i...
    if (carry)
    {
        err = FTOA_ERR_ROUND;
    }
    return err;
} //end ftoa_roundup

/******************************************************************************
FUNCTION: ftoa_minutes_to_deg_str()
convert decimal minutes string to degree decimal string.
The minutes will be less than 60 to give a fraction less than 1.
Assume a positive number.
This converts to 32-bit unsigned integers and performs the operation on this 
number to give 26 bits of precision for result
******************************************************************************/
int ftoa_minutes_to_deg_str(int abs_minutes, // 0-59 for number of minutes
                   float abs_frac,  // fraction of minute
                   char* buf,       // place to put string
                   int buf_len)     // length of buffer including null character
{
    int err = 0;
    const unsigned long uscale = 10000000UL; // 10^7
    char* ptr = buf;
    float scale = (float) (uscale);  // 10^7
    unsigned long lval = ((unsigned long) abs_minutes) * uscale;
    unsigned long lfrac = (unsigned long)(abs_frac * scale);
#if DEBUG_FTOA_MINUTES
    printf(" abs_minutes = %d, abs_frac = %.9f\n", abs_minutes, abs_frac);
    printf(" lval = %lu, lfrac = %lu\n", lval, lfrac);
#endif
    if (buf_len < (7+2))  // 7 digits . and null
    {
        return FTOA_ERR_TOO_LONG;
    }
    *ptr++ = '.';
    lval += lfrac;
    lval /= 6UL; // divide by 60 and multiply by 10 to shift to left
#if DEBUG_FTOA_MINUTES
    printf("starting lval = %lu\n", lval);
#endif
    int i;
    for (i = 0; i < 7; i++)
    {
        unsigned long tval = lval/uscale;
#if DEBUG_FTOA_MINUTES
        printf("tval = %lu\n", tval);
#endif
        lval -= tval*uscale;
        lval *= 10UL;  // shift digits to left
#if DEBUG_FTOA_MINUTES
        printf("next lval = %lu\n", lval);
#endif
        *ptr++ = '0' + tval; 
    }
    *ptr = '\0';
#if DEBUG_FTOA_MINUTES
    printf("minutes_to_deg_str: returns %s\n", buf);
#endif
    return err;

    
    
} // end ftoa_minutes_to_deg_str

