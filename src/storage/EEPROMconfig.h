#ifndef EECONFIG_H
#define EECONFIG_H

#include "math/Waypoint.h"

typedef uint16_t EEaddr;
typedef float EE_STORAGE_TYPE;
typedef float EE_LIST_TYPE;

/*
// Uno
#if defined(__AVR_ATmega328P__)
    const EEaddr EE_MAX = 1024;
// Mega
#elif defined(__AVR_ATmega2560__)
    const EEaddr EE_MAX = 4096;
// leonardo
#elif defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
    const EEaddr EE_MAX = 1024;
// other
#elif defined(__AVR_ATmega32U4__)
    const EEaddr EE_MAX = 1024;
// I wish there were a better way to do this *grumble*
#endif
*/
const EEaddr EE_MAX = 256;
//eelist type should be waypoint

const uint8_t NUM_STORED_RECORDS = 32;
const EEaddr EENULL         = 0;
const EEaddr EEaddrStart    = 1;
const EEaddr EE_LIST_START  = EEaddrStart + 
                              sizeof(EE_STORAGE_TYPE) * NUM_STORED_RECORDS;
const EEaddr EE_LIST_LENGTH = (EE_MAX -EE_LIST_START -1);

#endif
