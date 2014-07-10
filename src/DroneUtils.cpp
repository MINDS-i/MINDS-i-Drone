#include "DroneUtils.h"

CommManager::CommManager(HardwareSerial *inStream): stream(inStream),
		bufPos(0), waypoints(Protocol::MAX_WAYPOINTS), cachedTarget(0,0),
		targetIndex(0) {
}
void
CommManager::update(){
	while(stream->available()){
		uint8_t tmp = stream->read();
		if(buf[bufPos-1] == Protocol::END_BYTE[0] &&
				tmp == Protocol::END_BYTE[1]){
			bufPos--;  //hide first end byte
			processMessage(buf, bufPos);
			bufPos = 0;
		} else if(bufPos > BUFF_LEN){
			bufPos = 0;
		} else {
			buf[bufPos] = tmp;
			bufPos++;
		}
	}
}
void
CommManager::processMessage(uint8_t* data, uint8_t length){
	if(!Protocol::fletcher(data, length)) return;
	switch(length){
		case Protocol::WAYPOINT_MSG_LENGTH:
			{
				uint8_t tag = data[0];
				int32_t tmplat, tmplon;
				tmplat = ( ((uint32_t)data[1]<<24) | ((uint32_t)data[2]<<16) |
								((uint32_t)data[3]<<8) | ((uint32_t)data[4]) );
				tmplon = ( ((uint32_t)data[5]<<24) | ((uint32_t)data[6]<<16) |
								((uint32_t)data[7]<<8) | ((uint32_t)data[8]) );
				double lat = ((double)tmplat) / 100000.;
				double lon = ((double)tmplon) / 100000.;
				uint8_t pos = data[9];

				if(recieveWaypoint(tag, lat, lon, pos))
					sendConfirm(Protocol::fletcher16(data, length),stream);
			}
			break;
		case Protocol::COMMAND_MSG_LENGTH:
			{
				recieveCommand(data[0]);
				sendConfirm(Protocol::fletcher16(data, length),stream);
			}
			break;
		//the robot should never recieve either of these
		case Protocol::COMFIRM_MSG_LENGTH:
		case Protocol::DATA_MSG_LENGTH:
		default:
			break;
	}
}
void
CommManager::sendConfirm(uint16_t data, HardwareSerial *stream){
	byte datum[2] = {data>>8, data&0xff};
	stream->write(datum, 2);
	uint16_t sum = Protocol::fletcher16(datum, 2);
	stream->write(sum>>8);
	stream->write(sum&0xff);
	stream->write(Protocol::END_BYTE, 2);
}
boolean
CommManager::recieveWaypoint(uint8_t tag, double lat,
								double lon, uint8_t index){
	switch(tag){
		case Protocol::RECEIVE_TARGET:
			if(index > waypoints.size()-1) return false;
			setTargetIndex(index);
			break;
		case Protocol::ADD_WAYPOINT:
			if(index > waypoints.size()) return false;
			waypoints.add(index, Point(lat,lon));
			if(index == targetIndex) cachedTarget = getWaypoint(targetIndex);
			if(index <  targetIndex) advanceTargetIndex();
			break;
		case Protocol::CHANGE_WAYPOINT:
			if(index > waypoints.size()-1) return false;
			waypoints.set(index, Point(lat, lon));
			if(index == targetIndex) cachedTarget = getWaypoint(targetIndex);
			break;
		case Protocol::DELETE_WAYPOINT:
			if(index > waypoints.size()-1) return false;
			waypoints.remove(index);
			if(targetIndex >= index) retardTargetIndex();
			break;
	}
	return true;
}
void
CommManager::recieveCommand(uint8_t command){
	switch(command){
		case Protocol::LOOP_ON:
			isLooped = true;
			break;
		case Protocol::LOOP_OFF:
			isLooped = false;
			break;
		case Protocol::DELETE_LIST:
			waypoints.clear();
			setTargetIndex(0);
			break;
	}
}
inline Point
CommManager::getWaypoint(int index){
	if(index >= waypoints.size()) return Point(0,0);
	return waypoints.get(index);
}
int
CommManager::numWaypoints(){
	return waypoints.size();
}
bool
CommManager::loopWaypoints(){
	return isLooped;
}
void
CommManager::setTargetIndex(unsigned int index){
	targetIndex = index;
	cachedTarget = getWaypoint(targetIndex);
	sendTargetIndex();
}
void
CommManager::advanceTargetIndex(){
	targetIndex++;
	cachedTarget = getWaypoint(targetIndex);
	sendTargetIndex();
}
void
CommManager::retardTargetIndex(){
	targetIndex--;
	cachedTarget = getWaypoint(targetIndex);
	sendTargetIndex();
}
unsigned int
CommManager::getTargetIndex(){
	return targetIndex;
}
Point
CommManager::getTargetWaypoint(){
	return cachedTarget;
}
void
CommManager::sendTargetIndex(){
	uint8_t message[10];
	message[0] = Protocol::ROVER_TARGET;
	message[9] = targetIndex;
	Protocol::sendMessage(message, 10, stream);
}
void
CommManager::sendDataMessage(uint8_t tag, double data){
	uint8_t message[5];
	data *= 10000000;
	long ndata = (long)data;
	message[0] = tag;
	message[1] = (ndata>>24)&0xff;
	message[2] = (ndata>>16)&0xff;
	message[3] = (ndata>>8 )&0xff;
	message[4] = (ndata    )&0xff;
	Protocol::sendMessage(message, 5, stream);
}
void
CommManager::sendDataMessage(uint8_t tag, long data){
	data *= 10000000;
	uint8_t message[5];
	message[0] = tag;
	message[1] = (data>>24)&0xff;
	message[2] = (data>>16)&0xff;
	message[3] = (data>>8 )&0xff;
	message[4] = (data    )&0xff;
	Protocol::sendMessage(message, 5, stream);
}
void
CommManager::requestResync(){
	uint8_t message[1];
	message[0] = Protocol::SEND_WAYPOINT_LIST;
	Protocol::sendMessage(message, 1, stream);
}

//Serial port and commands specific to stock 3DR UBLOX GPS
void sendGPSMessage(uint8_t Type, uint8_t ID, uint16_t len, const uint8_t* buf){
  uint8_t header[4];
  uint8_t check[2] = {0,0};
  header[0] = Type;
  header[1] = ID;
  header[2] = (uint8_t) (len&0xff);
  header[3] = (uint8_t) len>>8;

  updateGPSchecksum(header, 4, check[0], check[1]);
  if(len > 0) updateGPSchecksum(buf, len, check[0], check[1]);

  Serial1.write((char)0xB5);
  Serial1.write((char)0x62);
  Serial1.write(header, 4);
  if(len > 0) Serial1.write(buf, len);
  Serial1.write(check, 2);
}

void updateGPSchecksum(const uint8_t *data, uint8_t len, uint8_t &c_a, uint8_t &c_b){
    while (len--) {
        c_a += *data;
        c_b += c_a;
        data++;
    }
}
