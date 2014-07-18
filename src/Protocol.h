#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <inttypes.h>
#include "Arduino.h"

namespace Protocol{
	const uint8_t  MAX_WAYPOINTS  	  = 200;
	const uint8_t  MAX_DATA_SLOTS     = 32;
	const uint16_t BAUD_RATE 		  = 9600;
	const float    FIXED_POINT_FACTOR = 0x100000;

	const boolean WAYPOINT_CONFIRM_REQ = true;
	const boolean DATA_CONFIRM_REQ     = false;

	const uint8_t DATA_LATITUDE  = 0x00;
	const uint8_t DATA_LONGITUDE = 0x01;
	const uint8_t DATA_HEADING   = 0x02;
	const uint8_t DATA_PITCH     = 0x03;
	const uint8_t DATA_ROLL      = 0x04;
	const uint8_t DATA_SPEED     = 0x05;
	const uint8_t DATA_VOLTAGE   = 0x06;
	const uint8_t DATA_TARGET    = 0x07;
	const uint8_t DATA_LOOPING   = 0x08;

	const uint8_t DATA_MSG 		  = 0x00;
	const uint8_t ADD_WAYPOINT    = 0x10;
	const uint8_t CHANGE_WAYPOINT = 0x11;
	const uint8_t DELETE_WAYPOINT = 0x12;
	const uint8_t CLEAR_WAYPOINT  = 0x13;
	const uint8_t CONFIRMATION    = 0x20;
	const uint8_t REQUEST_RESYNC  = 0x30;

	const uint8_t HEADER[] = {0x13, 0x37};
	const uint8_t HEADER_SIZE = 2;
	const uint8_t FOOTER[] = {0x7A };
	const uint8_t FOOTER_SIZE = 1;

	void sendMessage(uint8_t* data, int length, HardwareSerial *stream);

	//return a 16-bit fletcher checksum for an array of data
	uint16_t fletcher16(uint8_t* data, int length);

	//return true if an array has a valid fletcher checksum concatenated
	bool fletcher(uint8_t* data, int length);
}

#endif
