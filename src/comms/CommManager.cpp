#include "CommManager.h"

using namespace Protocol;
 
 CommManager::CommManager(HardwareSerial *inStream, Storage<float> *settings)
 :
		stream(inStream),
		bufPos(0),
		storage(settings),
		cachedTarget(0,0),
		isLooped(false),
		targetIndex(0),
		waypointsLooped(false),
		connectCallback(NULL),
		eStopCallback(NULL),
		chksumFailureCount(0),
		pktMismatchCount(0),
		pktFormatErrorCount(0),
		pktDataCount(0),
		pktDataLen(0),
		pktDecodeState(READ_HEADER1)
{
	waypoints = new SRAMlist<Waypoint>(MAX_WAYPOINTS);
}

void CommManager::update()
{
	while(stream->available())
	{
		uint8_t tmp = stream->read();
		buf[bufPos] = tmp;
		bufPos++;
		if(bufPos > BUFF_LEN)
		{ 
			bufPos = 0;
		}
		else if(rightMatch( buf, 	bufPos, HEADER, HEADER_SIZE))
		{
			bufPos = 0;
		}
		else if(rightMatch( buf, 	bufPos, FOOTER, FOOTER_SIZE))
		{
			processMessage(buf, bufPos-FOOTER_SIZE);
		}
	}
}


// void CommManager::update()
// {

// 	while(stream->available())
// 	{
// 		buf[bufPos] = stream->read();
// 		switch(pktDecodeState)
// 		{
// 			case READ_HEADER1:
// 	            if (buf[bufPos] == HEADER[0])
// 	            {	       	              
// 	              pktDecodeState=READ_HEADER2;
// 	            }
// 	            break;
//             case READ_HEADER2:             	
// 	            if (buf[bufPos] == HEADER[1])
// 	            {
// 	              pktDecodeState=READ_LABEL;
// 	            }
// 	            else
// 	            {
// 	              pktDecodeState=READ_HEADER1;            
// 	            }
// 	            bufPos=0;
// 	            break;
//  			case READ_LABEL:            	
// 	            pktDataLen=( buf[bufPos] & 0xf0 ) >> 4;
// 	            if (pktDataLen == 0)
// 	            {
// 	              pktDecodeState=READ_CHKSUM1;
// 	            }
// 	            else
// 	            {
// 	              pktDataCount=0;
// 	              pktDecodeState=READ_DATA;
// 	            }
// 	            bufPos++;
// 	            break;	        
//           	case READ_DATA:
// 	            pktDataCount++;
// 	            if (pktDataCount >= pktDataLen)
// 	              pktDecodeState = READ_CHKSUM1;
// 	          	bufPos++;
// 	            break;
// 	        case READ_CHKSUM1:    	                  
//         	    pktDecodeState = READ_CHKSUM2;
//         	    bufPos++;
//             	break;
//           	case READ_CHKSUM2:
//             	pktDecodeState = READ_FOOTER;
//             	bufPos++;
//             	break;
//           	case READ_FOOTER:
//             	if (buf[bufPos] == FOOTER[0])
//             	{
//               		processMessage(buf, bufPos-FOOTER_SIZE);	              	
// 	            }
// 	            else
// 	            {
// 	            	pktFormatErrorCount++;
// 	            }
// 	            pktDecodeState=READ_HEADER1;
// 	            bufPos=0;
// 	            break;

// 		}
// 	}

// }


//returns if the rightmost characters of lhs match rhs 
boolean CommManager::rightMatch(const uint8_t* lhs, const uint8_t llen ,const uint8_t* rhs, const uint8_t rlen)
{
	if(rlen > llen) 
		return false;

	for(int i=0; i < rlen; i++)
	{
		if( lhs[llen-i-1] != rhs[rlen-i-1] ) 
			return false;
	}
	return true;
}

void CommManager::processMessage(uint8_t* msg, uint8_t length)
{
	if(!fletcher(msg, length)) 
	{
		chksumFailureCount++;
		return;
	}
	messageType type = getMessageType(msg[0]);
	switch(type)
	{
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
	if(needsConfirmation(msg[0]))
	{
		sendConfirm(fletcher16(msg, length));
	}
}

//BA added for testing.  Should make handleWaypoint call this function to avoid 
//duplicate code.  Leaving for now
void CommManager::addWaypoint(float lat, float lng, uint8_t index, uint16_t alt )
{
	if(index > waypoints->size()) 
	{
		requestResync();
		return;
	}
	Waypoint data = Waypoint(lat, lng, Units::DEGREES, alt);
	//todo fix. Now we assume index is ok?
	//add would returns error but ignored
	waypoints->add(index, data);
	if(index <= getTargetIndex()) 
		advanceTargetIndex();
	if(index == getTargetIndex()) 
		cachedTarget = getWaypoint(index);

}

inline void CommManager::handleWaypoint(uint8_t* msg, uint8_t length)
{
	uint8_t subtype = getSubtype(msg[0]);
	byteConv lat, lon;
	for(int i=0; i<4; i++)
	{
		lat.bytes[3-i] = msg[1+i];
		lon.bytes[3-i] = msg[5+i];
	}
	uint16_t alt  = (((uint16_t)msg[9])<<8) | msg[10];
	Waypoint data = Waypoint(lat.f, lon.f, Units::DEGREES, alt);

	uint8_t  index = msg[11];
	switch(subtype)
	{
		case ADD:
			if(index > waypoints->size()) 
			{
				requestResync();
				return;
			}
			//todo fix. Now we assume index is ok?
			//add would returns error but ignored
			waypoints->add(index, data);
			if(index <= getTargetIndex()) 
				advanceTargetIndex();
			if(index == getTargetIndex()) 
				cachedTarget = getWaypoint(index);
			break;
		case ALTER:
			if(index >= waypoints->size()) 
			{
				requestResync();
				return;
			}
			waypoints->set(index, data);
			if(index == getTargetIndex()) 
				cachedTarget = getWaypoint(index);
			break;
		default:
			//todo packet subtype error
			break;
	}
}

inline void CommManager::handleData(uint8_t* msg, uint8_t length)
{
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

inline void CommManager::handleWord(uint8_t* msg, uint8_t length)
{
	uint8_t subtype = getSubtype(msg[0]);
	uint8_t a = msg[1];
	uint8_t b = msg[2];
	switch(subtype){
		case CONFIRMATION:
			//uint16_t sum = (((uint16_t)a)<<8) | b;
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

inline void CommManager::handleCommands(uint8_t a, uint8_t b)
{
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
			if(b < 0 || b >= waypoints->size())
			{
				requestResync();
				return;
			}
			waypoints->remove(b);
			if(b <= getTargetIndex()) 
				retardTargetIndex();
			if(waypoints->size()==0) 
				cachedTarget = Waypoint();
			break;
		case STATE_STOP:
			if (stateStopCallback != NULL) 
			{
				sendString("Stopping");
				stateStopCallback();
			}
			break;
		case STATE_START:
			if (stateStartCallback != NULL) 
			{
				sendString("Starting");
				stateStartCallback();
			}
			break;
	}
}

void CommManager::handleString(uint8_t* msg, uint8_t length)
{
	/*dead end for strings*/
}

Waypoint CommManager::getWaypoint(uint16_t index)
{
	if(index >= waypoints->size()) return Waypoint();
	return waypoints->get(index);
}

void CommManager::clearWaypointList()
{
	waypoints->clear();
}

void CommManager::sendConfirm(uint16_t digest)
{
	byte datum[3];
	datum[0] = buildMessageLabel(wordSubtype(CONFIRMATION));
	datum[1] = (digest>>8  );
	datum[2] = (digest&0xff);
	sendMessage(datum, 3, stream);
}

uint16_t CommManager::numWaypoints()
{
	return waypoints->size();
}

bool CommManager::loopWaypoints()
{
	return waypointsLooped;
}

void CommManager::setTargetIndex(uint16_t index)
{
	if(index >= waypoints->size()) return;
	targetIndex = index;
	cachedTarget = getWaypoint(index);
	sendCommand(commandType(TARGET), targetIndex);
}

void CommManager::advanceTargetIndex()
{
	setTargetIndex(getTargetIndex()+1);
}

void CommManager::retardTargetIndex()
{
	setTargetIndex(getTargetIndex()-1);
}

uint16_t CommManager::getTargetIndex()
{
	return targetIndex;
}

Waypoint CommManager::getTargetWaypoint()
{
	return cachedTarget;
}

void CommManager::setSetting(uint8_t id,   float input)
{
	storage->updateRecord(id, input);
	sendSetting(id, input);
}

void CommManager::inputSetting(uint8_t id, float input)
{
	storage->updateRecord(id, input);
}

float CommManager::getSetting(uint8_t id)
{
	return storage->getRecord(id);
}

void CommManager::requestResync()
{
	sendSyncMessage(Protocol::SYNC_REQUEST);
}

void CommManager::setConnectCallback(void (*call)(void))
{
	connectCallback = call;
}

void CommManager::setEStopCallback(void (*call)(void))
{
	eStopCallback = call;
}

void CommManager::setStateStopCallback(void (*call)(void))
{
	stateStopCallback = call;
}

void CommManager::setStateStartCallback(void (*call)(void))
{
	stateStartCallback = call;
}



void CommManager::onConnect()
{
	for(int i=0; i<MAX_SETTINGS; i++){
		sendSetting(i, getSetting(i));
	}
	if(connectCallback != NULL) connectCallback();
}

void CommManager::sendTelem(uint8_t id, float value)
{
	byteConv data;
	data.f = value;
	byte tmp[6] = {	buildMessageLabel(dataSubtype(TELEMETRY)),
					id,
					data.bytes[3], data.bytes[2],
					data.bytes[1], data.bytes[0], };
	Protocol::sendMessage(tmp, 6, stream);
}

void CommManager::sendState(uint8_t stateTypeId, uint8_t stateID)
{
	byte tmp[3] = { buildMessageLabel(wordSubtype(STATE)),
					stateTypeId,
					stateID,
				};
	Protocol::sendMessage(tmp,3,stream); 
}

void CommManager::sendSensor(uint8_t sensorTypeId, uint8_t sensorNum, float value)
{
	byteConv data;
	data.f = value;
	byte tmp[7] = {	buildMessageLabel(dataSubtype(SENSORS)),
					sensorTypeId,
					sensorNum,
					data.bytes[3], data.bytes[2],
					data.bytes[1], data.bytes[0], };
	Protocol::sendMessage(tmp, 7, stream);
}



void CommManager::sendSetting(uint8_t id, float value)
{
	byteConv data;
	data.f = value;
	byte tmp[6] = {	buildMessageLabel(dataSubtype(SETTING)),
					id,
					data.bytes[3], data.bytes[2],
					data.bytes[1], data.bytes[0], };
	Protocol::sendMessage(tmp, 6, stream);
}

void CommManager::sendCommand(uint8_t id, uint8_t data)
{
	byte tmp[3] = { buildMessageLabel(wordSubtype(COMMAND)), id, data };
	Protocol::sendMessage(tmp, 3, stream);
}

void CommManager::sendSyncMessage(uint8_t syncMsg)
{
	byte datum[3] = {buildMessageLabel(wordSubtype(SYNC)), syncMsg, 0};
	sendMessage(datum, 3, stream);
}

void CommManager::sendString(int type, const char* msg, uint8_t len)
{
	//defer to protocol for more complicated procedure
	Protocol::sendStringMessage(buildMessageLabel(stringSubtype(type))
									, msg, len, stream);
}

void CommManager::sendString(char const * msg)
{
	uint8_t len = strnlen(msg, 0xFF);
	sendString(Protocol::stringSubtype(STR_STATE), msg, len);
}

void CommManager::sendError(char const * msg)
{
	uint8_t len = strnlen(msg, 0xFF);
	sendString(Protocol::stringSubtype(ERROR), msg, len);
}
