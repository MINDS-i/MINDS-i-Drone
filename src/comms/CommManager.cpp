#include "CommManager.h"

using namespace Protocol;

CommManager::CommManager(HardwareSerial *inStream, Storage<float> *settings):
		stream(inStream), bufPos(0), cachedTarget(0,0), storage(settings),
		connectCallback(NULL) {
	waypoints = new SRAMlist<Point>(MAX_WAYPOINTS);
	sendSync();
}
void
CommManager::update(){
	while(stream->available()){
		uint8_t tmp = stream->read();
		buf[bufPos] = tmp;
		bufPos++;
		if(bufPos > BUFF_LEN) bufPos = 0;
		else if(rightMatch( buf, 	bufPos,
							HEADER, HEADER_SIZE)){
			bufPos = 0;
		}
		else if(rightMatch( buf, 	bufPos,
							FOOTER, FOOTER_SIZE)){
			processMessage(buf, bufPos-FOOTER_SIZE);
		}
	}
}
boolean //returns if the rightmost characters of lhs match rhs
CommManager::rightMatch(const uint8_t* lhs, const uint8_t llen,
						const uint8_t* rhs, const uint8_t rlen){
	if(rlen > llen) return false;
	for(int i=0; i < rlen; i++){
		if( lhs[llen-i-1] != rhs[rlen-i-1] ) return false;
	}
	return true;
}
void //the subtype split is a bit ugly right now
CommManager::processMessage(uint8_t* msg, uint8_t length){
	if(!fletcher(msg, length)) return;
	//if(length != getMessageLength(msg[0])) return; //debug this
	messageType type = getMessageType(msg[0]);
	uint8_t subtype  = getSubtype(msg[0]);
	switch(type){
		case STANDARD: {
			if (subtype == COMMAND){
				handleCommand(commandType(msg[1]), msg[2]);
			}
		} break;
		case SETTINGS: {
			if (subtype == POLL) {
				sendSetting(msg[1], getSetting(msg[1]));
				break;
			}
			byteConv conv;
			for(int i=0; i<4; i++) conv.bytes[3-i] = msg[2+i];
			inputSetting(msg[1], conv.f);
		} break;
		case WAYPOINT: {
			byteConv lat, lon;
			for(int i=0; i<4; i++){
				lat.bytes[3-i] = msg[1+i];
				lon.bytes[3-i] = msg[5+i];
			}
			uint16_t alt = (((uint16_t)msg[9])<<8) | msg[10];
			uint8_t  pos = msg[11];
			recieveWaypoint(waypointSubtype(subtype),
							pos, Point(lat.f, lon.f, alt) );
		} break;
		case PROTOCOL:{
			if (subtype == SYNC){
				onConnect();
				sendSyncResponse();
				break;
			}
			if (subtype == SYNC_RESP){
				onConnect();
				break;
			}
			//only other case is confirmation type - not responded to on drone
		} break;
		default:
			break;
	}
	if(needsConfirmation(type)){
		sendConfirm(fletcher16(msg, length));
	}
}
void
CommManager::clearWaypointList(){
	waypoints->clear();
}
void
CommManager::sendConfirm(uint16_t digest){
	byte datum[3] = {buildMessageLabel(protocolSubtype(CONFIRM),3),
						 digest>>8, digest&0xff};
	sendMessage(datum, 3, stream);
}
boolean
CommManager::recieveWaypoint(waypointSubtype type, uint8_t index, Point point){
	switch(type){
		case ADD:
			if(index > waypoints->size()) return false;
			waypoints->add(index, point);
			if(index <  getTargetIndex()) advanceTargetIndex();
			if(index == getTargetIndex()) cachedTarget = getWaypoint(index);
			break;
		case ALTER:
			if(index >= waypoints->size()) return false;
			waypoints->set(index, point);
			if(index == getTargetIndex()) cachedTarget = getWaypoint(index);
			break;
		case DELETE:
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
	return waypointsLooped;
}
void
CommManager::setTargetIndex(int index){
	if(index >= waypoints->size()) return;
	targetIndex = index;
	cachedTarget = getWaypoint(index);
}
void
CommManager::advanceTargetIndex(){
	if(targetIndex+1 >= waypoints->size()) return;
	targetIndex++;
	cachedTarget = getWaypoint(targetIndex);
}
void
CommManager::retardTargetIndex(){
	if(targetIndex <= 0) return;
	targetIndex--;
	cachedTarget = getWaypoint(targetIndex);
}
int
CommManager::getTargetIndex(){
	return targetIndex;
}
Point
CommManager::getTargetWaypoint(){
	return cachedTarget;
}
void
CommManager::setSetting(uint8_t id,   float input){
	storage->updateRecord(id, input);
	sendSetting(id, input);
}
void
CommManager::sendTelem(uint8_t id, float value){
	byteConv data;
	data.f = value;
	byte tmp[6] = {	buildMessageLabel(standardSubtype(TELEMETRY), 6),
					id,
					data.bytes[3], data.bytes[2],
					data.bytes[1], data.bytes[0], };
	Protocol::sendMessage(tmp, 6, stream);
}
//Functions below are private
void
CommManager::inputSetting(uint8_t id, float input){
	storage->updateRecord(id, input);
}
void
CommManager::sendSetting(uint8_t id, float value){
	byteConv data;
	data.f = value;
	byte tmp[6] = {	buildMessageLabel(settingsSubtype(SET), 6),
					id,
					data.bytes[3], data.bytes[2],
					data.bytes[1], data.bytes[0], };
	Protocol::sendMessage(tmp, 6, stream);
}
float
CommManager::getSetting(uint8_t id){
	return storage->getRecord(id);
}
void
CommManager::requestResync(){
	sendSync();
}
void
CommManager::setConnectCallback(void (*call)(void)){
	connectCallback = call;
}
void
CommManager::setEStopCallback(void (*call)(void)){
	eStopCallback = call;
}
void
CommManager::onConnect(){
	for(int i=0; i<MAX_SETTINGS; i++){
		sendSetting(i, getSetting(i));
	}
	if(connectCallback != NULL) connectCallback();
}
void
CommManager::handleCommand(commandType command, uint8_t data){
	switch(command){
		case ESTOP:
			if(eStopCallback != NULL) eStopCallback();
			break;
		case TARGET:
			setTargetIndex(data);
			break;
		case LOOPING:
			waypointsLooped = data!=0;
			break;
		case CLEAR_WAYPOINTS:
			clearWaypointList();
			break;
	}
}
void
CommManager::sendSync(){
	byte datum[1] = {buildMessageLabel(protocolSubtype(SYNC),1)};
	sendMessage(datum, 1, stream);
}
void
CommManager::sendSyncResponse(){
	byte datum[1] = {buildMessageLabel(protocolSubtype(SYNC_RESP),1)};
	sendMessage(datum, 1, stream);
}
void //Serial port and commands specific to stock 3DR UBLOX GPS
sendGPSMessage(uint8_t Type, uint8_t ID, uint16_t len, const uint8_t* buf){
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
void
updateGPSchecksum(const uint8_t *msg, uint8_t len, uint8_t &c_a, uint8_t &c_b){
    while (len--) {
        c_a += *msg;
        c_b += c_a;
        msg++;
    }
}

