#include "CommManager.h"

using namespace Protocol;

CommManager::CommManager(HardwareSerial *inStream, Storage<float> *settings):
		stream(inStream),
		bufPos(0),
		storage(settings),
		cachedTarget(0,0),
		isLooped(false),
		targetIndex(0),
		waypointsLooped(false),
		connectCallback(NULL),
		eStopCallback(NULL) {
	waypoints = new SRAMlist<Waypoint>(MAX_WAYPOINTS);
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
void
CommManager::processMessage(uint8_t* msg, uint8_t length){
	if(!fletcher(msg, length)) return;
	messageType type = getMessageType(msg[0]);
	switch(type){
		case WAYPOINT:
			handleWaypoint(msg,length);
			break;
		case DATA:
			handleData(msg,length);
			break;
		case WORD:
			handleWord(msg,length);
			break;
		case STRING:
			handleString(msg,length);
			break;
	}
	if(needsConfirmation(msg[0])){
		sendConfirm(fletcher16(msg, length));
	}
}

inline void
CommManager::handleWaypoint(uint8_t* msg, uint8_t length){
	uint8_t subtype = getSubtype(msg[0]);
	byteConv lat, lon;
	for(int i=0; i<4; i++){
		lat.bytes[3-i] = msg[1+i];
		lon.bytes[3-i] = msg[5+i];
	}
	uint16_t alt   = (((uint16_t)msg[9])<<8) | msg[10];
	uint8_t  index = msg[11];
	switch(subtype){
		case ADD:
			if(index > waypoints->size()) return;
			waypoints->add(index, Waypoint(lat.f, lon.f, alt));
			if(index <  getTargetIndex()) advanceTargetIndex();
			if(index == getTargetIndex()) cachedTarget = getWaypoint(index);
			break;
		case ALTER:
			if(index >= waypoints->size()) return;
			waypoints->set(index, Waypoint(lat.f, lon.f, alt));
			if(index == getTargetIndex()) cachedTarget = getWaypoint(index);
			break;
	}
}
inline void
CommManager::handleData(uint8_t* msg, uint8_t length){
	uint8_t subtype = getSubtype(msg[0]);
	uint8_t index = msg[1];
	byteConv conv;
	for(int i=0; i<4; i++) conv.bytes[3-i] = msg[2+i];
	switch(subtype){
		case TELEMETRY:
			//arduino doesn't keep track of received telemetry
			break;
		case SETTING:
			setSetting(index, conv.f);
			break;
	}
}
inline void
CommManager::handleWord(uint8_t* msg, uint8_t length){
	uint8_t subtype = getSubtype(msg[0]);
	uint8_t a = msg[1];
	uint8_t b = msg[2];
	uint16_t join = (((uint16_t)a)<<8) | b;
	switch(subtype){
		case CONFIRMATION:
			//arduino doesn't keep track of failed messages
			break;
		case SYNC:
			onConnect();
			if(a == Protocol::SYNC_REQUEST) sendSyncMessage(Protocol::SYNC_RESPOND);
			break;
		case COMMAND:
			handleCommands(a,b);
			break;
	}
}
inline void
CommManager::handleCommands(uint8_t a, uint8_t b){
	switch(a){
		case ESTOP:
			if(eStopCallback != NULL) eStopCallback();
			break;
		case TARGET:
			setTargetIndex(b);
			break;
		case LOOPING:
			waypointsLooped = (b!=0);
			break;
		case CLEAR_WAYPOINTS:
			sendString("Cleared Waypoints");
			waypoints->clear();
			break;
		case DELETE_WAYPOINT:
			waypoints->remove(b);
			if(b <= getTargetIndex()) retardTargetIndex();
			if(waypoints->size()==0) cachedTarget = Waypoint();
			break;
	}
}
inline void
CommManager::handleString(uint8_t* msg, uint8_t length){
	/*dead end for strings*/
}
inline Waypoint
CommManager::getWaypoint(uint16_t index){
	if(index >= waypoints->size()) return Waypoint();
	return waypoints->get(index);
}
void
CommManager::clearWaypointList(){
	waypoints->clear();
}
inline void
CommManager::sendConfirm(uint16_t digest){
	byte datum[3];
	datum[0] = buildMessageLabel(wordSubtype(CONFIRMATION));
	datum[1] = (digest>>8  );
	datum[2] = (digest&0xff);
	sendMessage(datum, 3, stream);
}
uint16_t
CommManager::numWaypoints(){
	return waypoints->size();
}
bool
CommManager::loopWaypoints(){
	return waypointsLooped;
}
void
CommManager::setTargetIndex(uint16_t index){
	if(index >= waypoints->size() || index < 0) return;
	targetIndex = index;
	cachedTarget = getWaypoint(index);
	sendCommand(commandType(TARGET), targetIndex);
}
void
CommManager::advanceTargetIndex(){
	setTargetIndex(getTargetIndex()+1);
}
void
CommManager::retardTargetIndex(){
	setTargetIndex(getTargetIndex()-1);
}
uint16_t
CommManager::getTargetIndex(){
	return targetIndex;
}
Waypoint
CommManager::getTargetWaypoint(){
	return cachedTarget;
}
void
CommManager::setSetting(uint8_t id,   float input){
	storage->updateRecord(id, input);
	sendSetting(id, input);
}
void
CommManager::inputSetting(uint8_t id, float input){
	storage->updateRecord(id, input);
}
float
CommManager::getSetting(uint8_t id){
	return storage->getRecord(id);
}
void
CommManager::requestResync(){
	sendSyncMessage(Protocol::SYNC_REQUEST);
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
CommManager::sendTelem(uint8_t id, float value){
	byteConv data;
	data.f = value;
	byte tmp[6] = {	buildMessageLabel(dataSubtype(TELEMETRY)),
					id,
					data.bytes[3], data.bytes[2],
					data.bytes[1], data.bytes[0], };
	Protocol::sendMessage(tmp, 6, stream);
}
void
CommManager::sendSetting(uint8_t id, float value){
	byteConv data;
	data.f = value;
	byte tmp[6] = {	buildMessageLabel(dataSubtype(SETTING)),
					id,
					data.bytes[3], data.bytes[2],
					data.bytes[1], data.bytes[0], };
	Protocol::sendMessage(tmp, 6, stream);
}
void
CommManager::sendCommand(uint8_t id, uint8_t data){
	byte tmp[3] = { buildMessageLabel(wordSubtype(COMMAND)), id, data };
	Protocol::sendMessage(tmp, 3, stream);
}
void
CommManager::sendSyncMessage(uint8_t syncMsg){
	byte datum[3] = {buildMessageLabel(wordSubtype(SYNC)), syncMsg, 0};
	sendMessage(datum, 3, stream);
}
void
CommManager::sendString(int type, const char* msg, uint8_t len){
	//defer to protocol for more complicated procedure
	Protocol::sendStringMessage(buildMessageLabel(stringSubtype(type))
									, msg, len, stream);
}
void
CommManager::sendString(char const * msg){
	uint8_t len = strnlen(msg, 0xFF);
	sendString(Protocol::stringSubtype(STATE), msg, len);
}
void
CommManager::sendError(char const * msg){
	uint8_t len = strnlen(msg, 0xFF);
	sendString(Protocol::stringSubtype(ERROR), msg, len);
}
