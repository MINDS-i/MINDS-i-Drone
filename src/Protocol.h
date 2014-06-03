#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <inttypes.h>
#include "Arduino.h"

namespace Protocol{
	const uint8_t DATA_LATITUDE  = 0x00;
	const uint8_t DATA_LONGITUDE = 0x01;
	const uint8_t DATA_HEADING   = 0x02;
	const uint8_t DATA_PITCH     = 0x03;
	const uint8_t DATA_ROLL      = 0x04;
	const uint8_t DATA_SPEED     = 0x05;
	const uint8_t DATA_DISTANCE  = 0x06;

	const uint8_t ADD_WAYPOINT    = 0x10;
	const uint8_t CHANGE_WAYPOINT = 0x11;
	const uint8_t DELETE_WAYPOINT = 0x12;
	const uint8_t ROVER_TARGET    = 0x13;
	const uint8_t RECEIVE_TARGET  = 0x14;

	const uint8_t SEND_WAYPOINT_LIST = 0x20;
	const uint8_t LOOP_ON     = 0x21;
	const uint8_t LOOP_OFF    = 0x22;
	const uint8_t DELETE_LIST = 0x23;

	const uint8_t COMFIRM_MSG_LENGTH  = 4;
	const uint8_t DATA_MSG_LENGTH     = 7;
	const uint8_t COMMAND_MSG_LENGTH  = 3;
	const uint8_t WAYPOINT_MSG_LENGTH = 12;
	const uint8_t MAX_MESSAGE_SIZE    = 12;
	const uint8_t MAX_WAYPOINTS		  = 200;

	const uint8_t END_BYTE[] = {0x17, 0x1f};

	void sendMessage(uint8_t* data, int length, HardwareSerial *stream);

	//return a 16-bit fletcher checksum for an array of data
	uint16_t fletcher16(uint8_t* data, int length);

	//return true if an array has a valid fletcher checksum concatenated
	bool fletcher(uint8_t* data, int length);
}

#endif
