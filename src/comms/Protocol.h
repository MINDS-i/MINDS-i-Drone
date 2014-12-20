#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <inttypes.h>
#include "Arduino.h"

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

namespace Protocol{
	const uint8_t MESSAGE_TYPE_MASK	= 0x03;

	enum messageType{	STANDARD = 0,
						SETTINGS = 1,
						WAYPOINT = 2,
						PROTOCOL = 3 };

	enum standardSubtype{	TELEMETRY	= 0,
							COMMAND		= 1 };

	enum waypointSubtype{	ADD		= 0,
							ALTER	= 1,
							DELETE	= 2 };

	enum settingsSubtype{	SET		= 0,
							POLL	= 1 };

	enum protocolSubtype{	SYNC	= 0,
							CONFIRM	= 1 };

	enum telemetryType{ LATITUDE	= 0,
						LONGITUDE	= 1,
						HEADING		= 2,
						PITCH		= 3,
						ROLL		= 4,
						SPEED		= 5,
						VOLTAGE		= 6 };

	enum commandType{	ESTOP			= 0,
						TARGET			= 1,
						LOOPING			= 2,
						CLEAR_WAYPOINTS = 3 };

	const uint8_t  MAX_WAYPOINTS	= 64;
	const uint8_t  MAX_SETTINGS		= 32;
	const uint16_t BAUD_RATE		= 9600;

	const boolean STANDARD_CONFIRM_REQ = false;
	const boolean SETTINGS_CONFIRM_REQ = true;
	const boolean WAYPOINT_CONFIRM_REQ = true;

	const uint8_t HEADER[] = {0x13, 0x37};
	const uint8_t HEADER_SIZE = 2;
	const uint8_t FOOTER[] = {0x7A};
	const uint8_t FOOTER_SIZE = 1;

	void sendMessage(uint8_t* data, int length, HardwareSerial *stream);

	//return a 16-bit fletcher checksum for an array of data
	uint16_t fletcher16(uint8_t* data, int length);

	//return true if an array has a valid fletcher checksum concatenated
	bool fletcher(uint8_t* data, int length);

	bool needsConfirmation(uint8_t label);

	messageType getMessageType(uint8_t label);

	uint8_t getSubtype(uint8_t label);

	uint8_t getMessageLength(uint8_t label);

	uint8_t buildMessageLabel(standardSubtype type, uint8_t length);
	uint8_t buildMessageLabel(settingsSubtype type, uint8_t length);
	uint8_t buildMessageLabel(waypointSubtype type, uint8_t length);
	uint8_t buildMessageLabel(protocolSubtype type, uint8_t length);
}

#endif
