#include "CommManager.h"

CommManager::CommManager(HardwareSerial *inStream):
		stream(inStream), bufPos(0), cachedTarget(0,0){
	for(int i=0; i<Protocol::MAX_DATA_SLOTS; i++) data[i] = 0;
	waypoints = new SRAMlist<Point>(Protocol::MAX_WAYPOINTS);
}
void
CommManager::update(){
	while(stream->available()){
		uint8_t tmp = stream->read();
		buf[bufPos] = tmp;
		bufPos++;
		if(bufPos > BUFF_LEN) bufPos = 0;
		else if(rightMatch( buf, 			  bufPos,
							Protocol::HEADER, Protocol::HEADER_SIZE)){
			bufPos = 0;
		}
		else if(rightMatch( buf, 	bufPos,
							Protocol::FOOTER, Protocol::FOOTER_SIZE)){
			processMessage(buf, bufPos-Protocol::FOOTER_SIZE);
		}
	}
}

boolean
CommManager::rightMatch(uint8_t* lhs, const uint8_t llen,
						const uint8_t* rhs, const uint8_t rlen){
	if(rlen > llen) return false;
	for(int i=0; i < rlen; i++){
		if( lhs[llen-i-1] != rhs[rlen-i-1] ) return false;
	}
	return true;
}

void
CommManager::processMessage(uint8_t* msg, uint8_t length){
	if(!Protocol::fletcher(msg, length)) return;
	uint8_t type = msg[0];
	switch(type){
		case Protocol::ADD_WAYPOINT:
		case Protocol::CHANGE_WAYPOINT:
		case Protocol::DELETE_WAYPOINT:
			{
				int32_t tmplat, tmplon;
				tmplat = ( ((uint32_t)msg[1]<<24) | ((uint32_t)msg[2]<<16) |
						   ((uint32_t)msg[3]<<8)  | ((uint32_t)msg[4]) );
				tmplon = ( ((uint32_t)msg[5]<<24) | ((uint32_t)msg[6]<<16) |
						   ((uint32_t)msg[7]<<8)  | ((uint32_t)msg[8]) );
				uint16_t alt = (((uint16_t)msg[9])<<8) | msg[10];
				double   lat = ((double)tmplat) / Protocol::FIXED_POINT_FACTOR;
				double   lon = ((double)tmplon) / Protocol::FIXED_POINT_FACTOR;
				uint8_t  pos = msg[11];

				recieveWaypoint(type, lat, lon, alt, pos);
				if(Protocol::WAYPOINT_CONFIRM_REQ)
					sendConfirm(Protocol::fletcher16(msg, length),stream);
			}
			break;
		case Protocol::DATA_MSG:
				{
					int32_t tmp;
					tmp = ( ((uint32_t)msg[2]<<24) | ((uint32_t)msg[3]<<16) |
							((uint32_t)msg[4]<<8)  | ((uint32_t)msg[5]) );
					inputData(msg[1], tmp);
					if(msg[1]==Protocol::DATA_TARGET){
						cachedTarget = getWaypoint(getInt(msg[1]));
					}
					if(Protocol::DATA_CONFIRM_REQ) {
						sendConfirm(Protocol::fletcher16(msg, length),stream);
					}
				}
			break;
		case Protocol::CONFIRMATION:
			//Currently, the rover only sends confirmations
			break;
		case Protocol::CLEAR_WAYPOINT:
			clearWaypointList();
			if(Protocol::WAYPOINT_CONFIRM_REQ)
					sendConfirm(Protocol::fletcher16(msg, length),stream);
			break;
		default:
			break;
	}
}
void
CommManager::clearWaypointList(){
	waypoints->clear();
}
void
CommManager::sendConfirm(uint16_t msg, HardwareSerial *stream){
	byte datum[3] = {Protocol::CONFIRMATION, msg>>8, msg&0xff};
	Protocol::sendMessage(datum, 3, stream);
}
boolean
CommManager::recieveWaypoint(uint8_t type, double lat, double lon, uint16_t alt,
										uint8_t index){
	switch(type){
		case Protocol::ADD_WAYPOINT:
			if(index > waypoints->size()) return false;
			waypoints->add(index, Point(lat,lon,alt));
			if(index <  getTargetIndex()) advanceTargetIndex();
			if(index == getTargetIndex()) cachedTarget = getWaypoint(index);
			break;
		case Protocol::CHANGE_WAYPOINT:
			if(index >= waypoints->size()) return false;
			waypoints->set(index, Point(lat, lon, alt));
			if(index == getTargetIndex()) cachedTarget = getWaypoint(index);
			break;
		case Protocol::DELETE_WAYPOINT:
			if(index >= waypoints->size()) return false;
			waypoints->remove(index);
			if(index <= getTargetIndex()) retardTargetIndex();
			if(waypoints->size()==0) cachedTarget = Point();
			break;
	}
	return true;
}
inline Point
CommManager::getWaypoint(int index){
	if(index >= waypoints->size()) return Point(0,0);
	return waypoints->get(index);
}
int
CommManager::numWaypoints(){
	return waypoints->size();
}
bool
CommManager::loopWaypoints(){
	return getBool(Protocol::DATA_LOOPING);
}
void
CommManager::setTargetIndex(int index){
	if(index >= waypoints->size()) return;
	setData(Protocol::DATA_TARGET, index);
	cachedTarget = getWaypoint(index);
}
void
CommManager::advanceTargetIndex(){
	int targetIndex = getInt(Protocol::DATA_TARGET);
	if(targetIndex+1 >= waypoints->size()) return;
	targetIndex++;
	setData(Protocol::DATA_TARGET, targetIndex);
	cachedTarget = getWaypoint(targetIndex);
}
void
CommManager::retardTargetIndex(){
	int targetIndex = getInt(Protocol::DATA_TARGET);
	if(targetIndex <= 0) return;
	targetIndex--;
	setData(Protocol::DATA_TARGET, targetIndex);
	cachedTarget = getWaypoint(targetIndex);
}
int
CommManager::getTargetIndex(){
	return getInt(Protocol::DATA_TARGET);
}
Point
CommManager::getTargetWaypoint(){
	return cachedTarget;
}
void
CommManager::setNewDataCallback( void	(*callback)(int) ){
	newDataCallback = callback;
}
//Functions below are private
void
CommManager::inputData (uint8_t id, int32_t input){
	data[id] = input;
	if(newDataCallback != 0) newDataCallback(id);
}
void
CommManager::sendData(uint8_t id){
	byte tmp[6] = {Protocol::DATA_MSG, id,
							(data[id]>>24)&0xff,
							(data[id]>>16)&0xff,
							(data[id]>>8 )&0xff,
							(data[id]    )&0xff	};
	Protocol::sendMessage(tmp, 6, stream);
}
void
CommManager::setData (uint8_t id,   float input){
	int32_t tmp = input * Protocol::FIXED_POINT_FACTOR;
	data[id] = tmp;
	sendData(id);
}
void
CommManager::setData (uint8_t id,  double input){
	int32_t tmp = input * Protocol::FIXED_POINT_FACTOR;
	data[id] = tmp;
	sendData(id);
}
void
CommManager::setData (uint8_t id,    long input){
	int32_t tmp = input * Protocol::FIXED_POINT_FACTOR;
	data[id] = tmp;
	sendData(id);
}
void
CommManager::setData (uint8_t id,     int input){
	int32_t tmp = input * Protocol::FIXED_POINT_FACTOR;
	data[id] = tmp;
	sendData(id);
}
void
CommManager::setData (uint8_t id, boolean input){
	int32_t tmp = (input)? 1*Protocol::FIXED_POINT_FACTOR : 0;
	data[id] = tmp;
	sendData(id);
}
int32_t
CommManager::getData (uint8_t id){
	return data[id];
}
int
CommManager::getInt  (uint8_t id){
	int32_t tmp = data[id]/Protocol::FIXED_POINT_FACTOR;
	return (int)tmp;
}
boolean
CommManager::getBool (uint8_t id){
	return data[id]!=0;
}
float
CommManager::getFloat(uint8_t id){
	float tmp = data[id];
	tmp /= Protocol::FIXED_POINT_FACTOR;
	return tmp;
}
void
CommManager::requestResync(){
	byte msg[3] = {Protocol::REQUEST_RESYNC};
	Protocol::sendMessage(msg, 1, stream);
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

  //make this be ignored when using unos

  Serial1.write((char)0xB5);
  Serial1.write((char)0x62);
  Serial1.write(header, 4);
  if(len > 0) Serial1.write(buf, len);
  Serial1.write(check, 2);
}

void updateGPSchecksum(const uint8_t *msg, uint8_t len, uint8_t &c_a, uint8_t &c_b){
    while (len--) {
        c_a += *msg;
        c_b += c_a;
        msg++;
    }
}

