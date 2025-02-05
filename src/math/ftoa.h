/******************************************************************************
FILE: ftoa.h
functions to help with converting float to ascii string
******************************************************************************/
#if !defined(FTOA_H)
#define FTOA_H

#define FTOA_ERR_BAD_VALUE (-1)
#define FTOA_ERR_TOO_LONG (-2)
#define FTOA_ERR_ROUND (-3)
#define FTOA_ERR_BAD_CHAR (-4)

int ftoa_frac(float x,             //[in] numberto convert
              char* buf,           // [in,out] place to put ASCII
              int len,             // [in] length of buffer including null
              int precision);      // [in] number of digits after decimal
int ftoa_roundup(char* buf,        // [in,out] pointer to buffer with number to round
                 int start_index); //[in] index to start with

int ftoa_minutes_to_deg_str(int abs_minutes, // 0-59 for number of minutes
                            float abs_frac,  // fraction of minute
                            char* buf,       // place to put string
                            int buf_len);    // length of buffer including null character
#endif
