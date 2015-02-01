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

//GPS_SETUP only for legacy support
const static uint8_t GPS_SETUP[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01, 0xFF, 0x18,
						0xB5, 0x62, 0x06, 0x17, 0x04, 0x00, 0x00, 0x23, 0x00, 0x00, 0x44, 0x52,
						0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x8A};
const static uint8_t Pedestrian_Mode[] = {0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00,
                            			  0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                             			  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
                             			  0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C,
                             		      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const static uint8_t GPRMC_On[] = { 0xF0, 0x04, 0x01 };
const static uint8_t CFG_NMEA[] = { 0x00, 0x23, 0x00, 0x00 };
const static uint8_t CFG_PRT[] =  { 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08,
                       				0x00, 0x00, 0x00, 0x96, 0x00, 0x00,
                       				0x07, 0x00, 0x02, 0x00, 0x00, 0x00,
                       				0x00, 0x00};

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
	boolean 			isLooped;
	List<Waypoint>*		waypoints;
	Storage<float>*		storage;
	Waypoint   			cachedTarget;
	uint16_t 			targetIndex;
	bool				waypointsLooped;
	void (*connectCallback)(void);
	void (*eStopCallback)(void);
public:
	CommManager(HardwareSerial *inStream, Storage<float> *settings);
	Waypoint getWaypoint(int index);
	Waypoint getTargetWaypoint();
	void  	update();
	void  	requestResync();
	void	sendTelem(uint8_t id, float value);
	void    setSetting(uint8_t id,   float input);
	float   getSetting(uint8_t id);
	int     getTargetIndex();
	void    setTargetIndex(int index);
	int 	numWaypoints();
	bool 	loopWaypoints();
	void 	clearWaypointList();
	void    advanceTargetIndex();
	void    retardTargetIndex();
	void	setConnectCallback(void (*call)(void));
	void	setEStopCallback(void (*call)(void));
private:
	void	onConnect();
	void	sendCommand(uint8_t id, uint8_t data);
	void	handleCommand(commandType command, uint8_t data);
	void	sendSync();
	void	sendSyncResponse();
	void    sendSetting(uint8_t id, float value);
	void    inputSetting(uint8_t id, float input);
	void    sendTargetIndex();
	void    processMessage(uint8_t* msg, uint8_t length);
	void    sendConfirm(uint16_t digest);
	boolean rightMatch(const uint8_t* lhs, const uint8_t llen,
					   const uint8_t* rhs, const uint8_t rlen);
	boolean recieveWaypoint(waypointSubtype type, uint8_t index, Waypoint point);
};

void sendGPSMessage(uint8_t Type, uint8_t ID, uint16_t len, const uint8_t* buf);
void updateGPSchecksum(const uint8_t *msg, uint8_t len, uint8_t &c_a, uint8_t &c_b);
#endif
