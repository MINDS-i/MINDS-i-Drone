#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "Arduino.h"
#include "storage/EEPROMconfig.h"
#include <inttypes.h>

/* messages are minumum of 3 bytes, max of 19 bytes, constructed as
    HEADER :(label[1 bytes] : data[length bytes] : checksum [2 bytes]): FOOTER
    label = legth[4 bits] : subType[2 bits] : type[2 bits]
    checksums calculated over everything except the checksum
    types and subtypes listed as enums below.
    some messages need confirmations, dictated by constants below
    confirmation messages contain only the checksum of the entire message
        being confirmed

    sync messages can be sent or received
    receiving a sync message should result in:
        the sending of a sync message
        the activation of any "on connection" commands

    the dashboard will send all waypoins "on connection"
    the drone will upload settings "on connection"
*/

namespace Protocol {
enum messageType { WAYPOINT = 0, DATA = 1, WORD = 2, STRING = 3 };

enum waypointSubtype {
    ADD = 0,
    ALTER = 1,
};

enum dataSubtype { TELEMETRY = 0, SETTING = 1, SENSORS = 2, INFO = 3 };

enum wordSubtype { CONFIRMATION = 0, SYNC = 1, COMMAND = 2, STATE = 3 };

enum stringSubtype { ERROR = 0, STR_STATE = 1 };

enum telemetryType {
    LATITUDE = 0,
    LONGITUDE = 1,
    HEADING = 2,
    PITCH = 3,
    ROLL = 4,
    GROUNDSPEED = 5,
    VOLTAGE = 6,
    AMPERAGE = 7,
    ALTITUDE = 8,
    RDROLL = 9,
    RDPITCH = 10,
    RDTHROTTLE = 11,
    RDYAW = 12,
    RDSWITCH = 13,
    RDAUX2 = 14,
    HOMELATITUDE = 15,
    HOMELONGITUDE = 16,
    HOMEALTITUDE = 17,
    DELTAALTITUDE = 18,
    GPSNUMSAT = 19,
    GPSHDOP = 20,
    HEADING_LOCK = 21
};
enum sensorType {
    OBJDETECT_SONIC = 0,
    OBJDETECT_BUMPER = 1,
};

enum infoType {
    APM_VERSION = 0,
};

enum commandType {
    ESTOP = 0,
    TARGET = 1,
    LOOPING = 2,
    CLEAR_WAYPOINTS = 3,
    DELETE_WAYPOINT = 4,
    STATE_STOP = 5,
    STATE_START = 6,
    BUMPER_DISABLE = 7,
    BUMPER_ENABLE = 8,
    SETTINGS_RESET = 9
};

enum stateType {
    APM_STATE = 0,
    DRIVE_STATE = 1,
    AUTO_STATE = 2,
    AUTO_FLAGS = 3,
    GPS_STATE = 4

};

const uint8_t MAX_WAYPOINTS = 64;
const uint8_t MAX_SETTINGS = NUM_STORED_RECORDS; // taken from eepromconfig
const uint16_t BAUD_RATE = 57600;
const uint16_t U16_FIXED_FACTOR = 256;

const uint8_t SYNC_REQUEST = 0x00;
const uint8_t SYNC_RESPOND = 0x01;

const uint8_t HEADER[] = {0x13, 0x37};
const uint8_t HEADER_SIZE = 2;
const uint8_t FOOTER[] = {0x9A};
const uint8_t FOOTER_SIZE = 1;

void sendStringMessage(uint8_t label, const char* msg, int length, HardwareSerial* stream);
void sendMessage(uint8_t* data, int length, HardwareSerial* stream);

// return a 16-bit fletcher checksum for an array of data
uint16_t fletcher16(uint8_t const* data, int length);
uint16_t fletcher16_resume(uint8_t const* data, int length, uint16_t lastResult);

// return true if an array has a valid fletcher checksum concatenated
bool fletcher(uint8_t* data, int length);

bool needsConfirmation(uint8_t label);

messageType getMessageType(uint8_t label);

uint8_t getSubtype(uint8_t label);

uint8_t buildMessageLabel(waypointSubtype type);
uint8_t buildMessageLabel(dataSubtype type);
uint8_t buildMessageLabel(wordSubtype type);
uint8_t buildMessageLabel(stringSubtype type);

} // namespace Protocol

#endif
