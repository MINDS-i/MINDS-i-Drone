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
}
