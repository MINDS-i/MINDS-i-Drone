#ifndef DRONEUTILS_H
#define DRONEUTILS_H

#include "Arduino.h"
#include <inttypes.h>

#include "comms/NMEA.h"
#include "comms/DroneProtocol.h"
#include "decoder/Protocol.h"
#include "math/GreatCircle.h"
#include "math/Waypoint.h"
#include "storage/List.h"
#include "storage/SRAMlist.h"
#include "storage/Storage.h"
#include "util/byteConv.h"

using namespace DroneProtocol;

const uint8_t BUFF_LEN = 32;

class CommManager{
public:
	CommManager(Stream *inStream, Storage<float> *settings);
	/** parse new data that has arrived */
	void  	 update();
	/**	Set the callcack to fire when an initial connection SYNC is received */
	void	 setConnectCallback(void (*call)(void));
	/** Set the callback to fire when ESTOP command is received */
	void	 setEStopCallback(void (*call)(void));
	/**
	 * Return weather the robot should stop or loop around when the last
	 * waypoint is reached
	 */
	bool 	 loopWaypoints();
	/** return the value of setting number `id` */
	float    getSetting(uint8_t id);
	/** retrieve the index of the rover's current target */
	uint16_t getTargetIndex();
	/** retrieve the number of waypoints in the rover's path */
	uint16_t numWaypoints();
	/** retrieve the Waypoint located at the rover's current target */
	Waypoint getTargetWaypoint();
	/** retrieve the Waypoint `index` (0 based) in the rover's current path */
	Waypoint getWaypoint(uint16_t index);
	/** Transmit telemetry type `id` equals `value` to the Dashboard */
	void	 sendTelem(uint8_t id , float value);
	/** Clear the local Waypoint List */
	void 	 clearWaypointList();
	/** Request that the Dashboard resynchronize settings and waypoints */
	void  	 requestResync();
	/**
	 *  Set the target to the next waypoint in the list and notify
	 *  the Dashboard
	 */
	void     advanceTargetIndex();
	/**
	 *  Set the target to the previous waypoint in the list and notify
	 *  the Dashboard
	 */
	void     retardTargetIndex();
	/**
	 * Set the target to the `index` (0 based) waypoint in the list
	 * notifies the dashboard
	 */
	void     setTargetIndex(uint16_t index);
	/** Set setting number `id` to the value `input`, notifing the dashboard */
	void     setSetting(uint8_t id,   float input);
	/** send a message with `msg` payload and `type` in the type field */
	void     sendString(int type, const char* msg, uint8_t len);
	/** Send a string to be displayed in the dashboard */
	void 	 sendString(char const * msg);
	/** Send an error message to be displayed in the dashboard */
	void 	 sendError(char const * msg);
private:
	Stream 			    *stream;
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
	CommManager(const CommManager& copy); //intentionally not implemented
	boolean recieveWaypoint(waypointSubtype type, uint8_t index, Waypoint point);
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
