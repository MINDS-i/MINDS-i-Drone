#ifndef DRONEUTILS_H
#define DRONEUTILS_H

#include "Arduino.h"
#include <inttypes.h>

#include "comms/NMEA.h"
#include "comms/Protocol.h"
#include "math/SpatialMath.h"
#include "math/Waypoint.h"
#include "storage/List.h"
#include "storage/SRAMlist.h"
#include "storage/Storage.h"
#include "util/byteConv.h"


#define extLogger true

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

 enum telemetryType{
	READ_HEADER1 = 0,
	READ_HEADER2,
	READ_LABEL,
	READ_DATA,
	READ_CHKSUM1,
	READ_CHKSUM2,
	READ_FOOTER
};

class CommManager
{
	HardwareSerial 		*stream;
	uint8_t 			buf[BUFF_LEN];
	uint8_t 			bufPos;
	Storage<float>*		storage;
	List<Waypoint>*		waypoints;
	Waypoint   			cachedTarget;
	boolean 			isLooped;
	uint16_t 			targetIndex;
	bool				waypointsLooped;

	//stats
	uint32_t			chksumFailureCount;
	uint32_t			pktMismatchCount;	
	uint32_t			pktFormatErrorCount;

	uint8_t 			pktDataCount;
	uint8_t				pktDataLen;
	uint8_t 			pktDecodeState;

	void (*connectCallback)(void);
	void (*eStopCallback)(void);
	void (*stateStopCallback)(void);
	void (*stateStartCallback)(void);
	void (*bumperDisableCallback)(void);	
	void (*bumperEnableCallback)(void);
	void (*settingsResetCallback)(void);

	void (*versionCallback)(void);

public:
	CommManager(HardwareSerial *inStream, Storage<float> *settings);
	bool 	 loopWaypoints();
	float    getSetting(uint8_t id);
	uint16_t getTargetIndex();
	uint16_t numWaypoints();
	void	 sendTelem(uint8_t id , float value);
	void 	 sendState(uint8_t stateTypeId, uint8_t stateID);
	void     sendSensor(uint8_t sensorTypeId, uint8_t sensorNum, uint32_t value);
	void 	 sendVersion(uint8_t version_major, uint8_t version_minor, uint8_t version_rev);

	void	 setConnectCallback(void (*call)(void));
	void	 setEStopCallback(void (*call)(void));
	void	 setStateStopCallback(void (*call)(void));
	void	 setStateStartCallback(void (*call)(void));	
	void	 setBumperDisableCallback(void (*call)(void));	
	void	 setBumperEnableCallback(void (*call)(void));
	void     setSettingsResetCallback(void (*call)(void));
	void 	 setVersionCallback(void (*call)(void));

	void 	 clearWaypointList();
	void  	 requestResync();
	void  	 update();
	void     advanceTargetIndex();
	void     retardTargetIndex();
	void     setSetting(uint8_t id,   float input);
	void     setTargetIndex(uint16_t index);
	void     sendString(int type, const char* msg, uint8_t len);
	void 	 sendString(char const * msg);
	void 	 sendError(char const * msg);
	Waypoint getTargetWaypoint();
	Waypoint getWaypoint(uint16_t index);

	void addWaypoint(float lat, float lng, uint8_t index, uint16_t alt );

private:
	CommManager(const CommManager& copy); //intentionally not implemented

    void    defaultVersionCallback();

	//No implementation? //boolean recieveWaypoint(waypointSubtype type, uint8_t index, Waypoint point);
	boolean rightMatch(const uint8_t* lhs, const uint8_t llen,
					   const uint8_t* rhs, const uint8_t rlen);
	void	onConnect();
	void	sendCommand(uint8_t id, uint8_t data);
	void	sendSyncMessage(uint8_t syncMsg);
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
