#include "Protocol.h"
#include <inttypes.h>
#include "Arduino.h"

namespace Protocol
{
	uint16_t fletcher16(uint8_t const *data, int length)
	{
		return fletcher16_resume(data, length, 0xFFFF);
	}

	uint16_t fletcher16_resume(uint8_t const *data, int length, uint16_t lastResult)
	{
			uint16_t Asum, Bsum;
			Asum = lastResult & 0xff;
			Bsum = (lastResult>>8) & 0xff;
			//Asum = 0xff; Bsum = 0xff;
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
	
	bool fletcher(uint8_t* data, int length)
	{
		uint16_t foundSum = data[length-2]<<8 | data[length-1];
		uint16_t calcSum = fletcher16(data, length-2);
		return foundSum==calcSum;
	}

	void sendMessage(uint8_t* data, int length, HardwareSerial *stream)
	{
		uint16_t sum = Protocol::fletcher16(data, length);
		stream->write(HEADER, HEADER_SIZE);
		stream->write(data,length);
		stream->write(sum>>8);
		stream->write(sum&0xff);
		stream->write(FOOTER, FOOTER_SIZE);
	}

	void sendStringMessage(uint8_t label, const char * msg, int len, HardwareSerial* stream)
	{
		uint16_t labelSum = Protocol::fletcher16(&label, 1);
		uint16_t sum = Protocol::fletcher16_resume((uint8_t const*)msg, len, labelSum);
		stream->write(HEADER, HEADER_SIZE);
		stream->write(label);
		stream->write(msg,len);
		stream->write(sum>>8);
		stream->write(sum&0xff);
		stream->write(FOOTER, FOOTER_SIZE);
	}

	bool needsConfirmation(uint8_t label)
	{
		uint8_t type = getMessageType(label);
		uint8_t subtype = getSubtype(label);

		if(type == WAYPOINT) return true;
		if(type == DATA && subtype == SETTING) return true;
		if(type == WORD && subtype == COMMAND) return true;

		return false;
	}

	messageType getMessageType(uint8_t label)
	{
		return (messageType) (label & 0x0F);
	}

	uint8_t getSubtype(uint8_t label)
	{
		return (label>>4) & 0x0F;
	}

	uint8_t buildMessageLabel(waypointSubtype subtype)
	{
		if(subtype > 0x0F) return 0;
		return (subtype<<4) | messageType(WAYPOINT);
	}
	uint8_t buildMessageLabel(dataSubtype subtype)
	{
		if(subtype > 0x0F) return 0;
		return (subtype<<4) | messageType(DATA);
	}
	uint8_t buildMessageLabel(wordSubtype subtype)
	{
		if(subtype > 0x0F) return 0;
		return (subtype<<4) | messageType(WORD);
	}
	uint8_t buildMessageLabel(stringSubtype subtype)
	{
		if(subtype > 0x0F) return 0;
		return (subtype<<4) | messageType(STRING);
	}
}
