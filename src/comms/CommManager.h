#ifndef DRONEUTILS_H
#define DRONEUTILS_H

#include "Arduino.h"
#include <inttypes.h>

#include "comms/NMEA.h"
#include "comms/Protocol.h"
#include "math/GreatCircle.h"
#include "math/Waypoint.h"
#include "storage/List.h"
#include "storage/SRAMlist.h"
#include "storage/Storage.h"
#include "util/byteConv.h"

using namespace Protocol;

const uint8_t BUFF_LEN = 32;

//Settings -- container supplied by outside world
//write setting
//read setting

//Messages -- send and forget message passing system
//send message
//attach message callback

//Waypoints -- uses self contained List implementation
//interface unchanged for now


class CommManager{
	HardwareSerial 		*stream;
	uint8_t 			buf[BUFF_LEN];
	uint8_t 			bufPos;
	Storage<float>*		storage;
	List<Waypoint>*		waypoints;
	Waypoint   			cachedTarget;
	boolean 			isLooped;
	uint16_t 			targetIndex;
	bool				waypointsLooped;
	void (*connectCallback)(void);
	void (*eStopCallback)(void);
public:
	CommManager(HardwareSerial *inStream, Storage<float> *settings);
	bool 	 loopWaypoints();
	float    getSetting(uint8_t id);
	uint16_t getTargetIndex();
	uint16_t numWaypoints();
	void	 sendTelem(uint8_t id , float value);
	void	 setConnectCallback(void (*call)(void));
	void	 setEStopCallback(void (*call)(void));
	void 	 clearWaypointList();
	void  	 requestResync();
	void  	 update();
	void     advanceTargetIndex();
	void     retardTargetIndex();
	void     setSetting(uint8_t id,   float input);
	void     setTargetIndex(uint16_t index);
	void 	 sendString(const char* msg , uint8_t len);
	Waypoint getTargetWaypoint();
	Waypoint getWaypoint(uint16_t index);
private:
	boolean recieveWaypoint(waypointSubtype type, uint8_t index, Waypoint point);
	boolean rightMatch(const uint8_t* lhs, const uint8_t llen,
					   const uint8_t* rhs, const uint8_t rlen);
	void	onConnect();
	void	sendCommand(uint8_t id, uint8_t data);
	void	sendSync();
	void	sendSyncResponse();
	void    inputSetting(uint8_t id, float input);
	void    processMessage(uint8_t* msg, uint8_t length);
	void    sendConfirm(uint16_t digest);
	void    sendSetting(uint8_t id, float value);
	void    sendTargetIndex();
	void    handleCommands(uint8_t a   , uint8_t b);
	void    handleData(uint8_t* msg    , uint8_t length);
	void    handleString(uint8_t* msg  , uint8_t length);
	void    handleWaypoint(uint8_t* msg, uint8_t length);
	void    handleWord(uint8_t* msg    , uint8_t length);
};

void sendGPSMessage(uint8_t Type, uint8_t ID, uint16_t len, const uint8_t* buf);
void updateGPSchecksum(const uint8_t *msg, uint8_t len, uint8_t &c_a, uint8_t &c_b);
#endif
