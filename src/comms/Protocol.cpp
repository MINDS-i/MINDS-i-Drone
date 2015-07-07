#include "Protocol.h"
#include <inttypes.h>
#include "Arduino.h"

namespace Protocol{
	uint16_t
	fletcher16(uint8_t* data, int length){
			uint16_t Asum, Bsum;
			Asum = 0xff; Bsum = 0xff;
			while(length){
				uint8_t tlen = (length > 20) ? 20 : length;
				length -= tlen;
				do{
					Bsum += Asum += *data++;
				} while (--tlen);
				Asum = (Asum & 0xff) + (Asum >> 8);
				Bsum = (Bsum & 0xff) + (Bsum >> 8);
			}
			Asum = (Asum & 0xff) + (Asum >> 8);
			Bsum = (Bsum & 0xff) + (Bsum >> 8);
			return (Bsum << 8) | Asum;
	}
	bool
	fletcher(uint8_t* data, int length){
		uint16_t foundSum = data[length-2]<<8 | data[length-1];
		uint16_t calcSum = fletcher16(data, length-2);
		return foundSum==calcSum;
	}
	void
	sendMessage(uint8_t* data, int length, HardwareSerial *stream){
		uint16_t sum = Protocol::fletcher16(data, length);
		stream->write(HEADER, HEADER_SIZE);
		stream->write(data,length);
		stream->write(sum>>8);
		stream->write(sum&0xff);
		stream->write(FOOTER, FOOTER_SIZE);
	}
	bool
	needsConfirmation(uint8_t label){
		messageType type = getMessageType(label);
		switch(type){
			case STANDARD:
				return STANDARD_CONFIRM_REQ;
				break;
			case SETTINGS:
				return SETTINGS_CONFIRM_REQ;
				break;
			case WAYPOINT:
				return WAYPOINT_CONFIRM_REQ;
				break;
			case PROTOCOL:
				return false;
			default:
				return false;
		}
	}
	messageType getMessageType(uint8_t label){
		return (messageType) (label & 0x0F);
	}
	uint8_t getSubtype(uint8_t label){
		return (label>>4) & 0x0F;
	}
	uint8_t buildMessageLabel(standardSubtype subtype){
		if(subtype > 0x0F) return 0;
		return (subtype<<4) | messageType(STANDARD);
	}
	uint8_t buildMessageLabel(settingsSubtype subtype){
		if(subtype > 0x0F) return 0;
		return (subtype<<4) | messageType(SETTINGS);
	}
	uint8_t buildMessageLabel(waypointSubtype subtype){
		if(subtype > 0x0F) return 0;
		return (subtype<<4) | messageType(WAYPOINT);
	}
	uint8_t buildMessageLabel(protocolSubtype subtype){
		if(subtype > 0x0F) return 0;
		return (subtype<<4) | messageType(PROTOCOL);
	}
}
