#ifndef UM7_H
#define UM7_H
#include "Arduino.h"
class UM7{
private:
	static const uint8_t HEADER_SIZE = 3;
	static const uint8_t PACKET_HEADER[HEADER_SIZE];
	static const uint8_t MAX_BUFFER = 5+64+2; //header, max body size, and checksum footer
	uint8_t buffer[UM7::MAX_BUFFER];
	uint8_t headerIndex;
	uint8_t bufferIndex;
	HardwareSerial* stream;
	void (*updateData)(uint8_t address, uint32_t data);
	bool parsePacketTo(uint8_t* buf, uint8_t size);
	void splitData(uint8_t address, uint8_t batch, uint8_t* buf);
public:
	static const double FIXED_CONVERSION;
	static const uint8_t QUAT_AB    = 0x6D;
	static const uint8_t QUAT_CD    = 0x6E;
	static const uint8_t QUAT_TIME  = 0x6F;
	static const uint8_t EULER_TIME = 0x74;

	UM7(HardwareSerial* inStream, void (*callback)(uint8_t, uint32_t)):
			stream(inStream), updateData(callback) {}
	void sendPacket(uint8_t PT, uint8_t ADDR, uint8_t* data, uint8_t len);
	void sendPacket(uint8_t PT, uint8_t ADDR);
	void update();
};
const double UM7::FIXED_CONVERSION = 29789.09091;
void
UM7::sendPacket(uint8_t PT, uint8_t ADDR, uint8_t* data, uint8_t len){
	uint8_t ptSize = len + 4;//PT, ADDR, 16 bit check
	uint8_t packet[ptSize];
	uint16_t check = 0;
	packet[0] = PT;
	packet[1] = ADDR;
	for(int i=0; i<len; i++) packet[i+2] = data[i];
	for(int i=0; i<UM7::HEADER_SIZE; i++) check += UM7::PACKET_HEADER[i];
	for(int i=0; i<ptSize-2; i++) 	 check += packet[i];
	packet[ptSize-2] = (uint8_t)(check>>8);
	packet[ptSize-1] = (uint8_t)(check &0xff);
	stream->write(UM7::PACKET_HEADER, UM7::HEADER_SIZE);
	stream->write(packet, ptSize);
	stream->print('\n');
}

const uint8_t UM7::PACKET_HEADER[HEADER_SIZE] = {'s','n','p'};

void
UM7::sendPacket(uint8_t PT, uint8_t ADDR){
	sendPacket(PT, ADDR, (uint8_t*) 0, 0);
}

void
UM7::update(){
	while(stream->available()){
		uint8_t tmp = stream->read();

		if(headerIndex == UM7::HEADER_SIZE){
			headerIndex = 0;
			bufferIndex = 0;
		} else if(tmp == UM7::PACKET_HEADER[headerIndex]) {
			headerIndex++;
			continue;
		} else {
			headerIndex = 0;
		}

		buffer[bufferIndex] = tmp;
		bufferIndex++;

		uint8_t batch = ((buffer[0]>>6)&0x1) * (buffer[0]>>2)&0xF;
		if(batch == 0 && (buffer[0]&0x80)!=0 ) batch = 1; //implied batch of one
		uint8_t expectedSize = 4 + 4*batch;

		if(bufferIndex >= UM7::MAX_BUFFER) bufferIndex = 0;
		else if(bufferIndex == expectedSize) parsePacketTo(buffer,bufferIndex);
	}
}

bool
UM7::parsePacketTo(uint8_t* buf, uint8_t size){
	uint16_t check = 0;
	for(int i=0; i<UM7::HEADER_SIZE; i++) check += UM7::PACKET_HEADER[i];
	for(int i=0; i<size-2;           i++) check += buf[i];
	uint16_t found = ((uint16_t)(buf[size-2]<<8))|((uint16_t)buf[size-1]);
	if(check != found){
		return false; //bad check sum; ignore
	}
	if(! (buf[0]>>7)&0x1 ){
		return true; //not a data transmission
	}
	uint8_t batch = ((buf[0]>>6)&0x1) * (buf[0]>>2)&0xF; //byte 6 = use batch
	if(batch == 0 && size == 8) batch = 1; //No batch, just one address
	splitData(buf[1], batch, buf+2); //Send the buffer starting at the data
	return true;
}

void
UM7::splitData(uint8_t address, uint8_t batch, uint8_t* buf){
	for(int i=0; i<batch; i++){
		uint32_t data = 0;
		for(int j=0; j<4; j++) {
			data |= ( ((uint32_t) buf[4*i+j])<<(8*(3-j)) );
		}
		updateData(address+i, data);
	}
}
#endif
