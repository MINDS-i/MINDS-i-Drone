#include "Arduino.h"
#include <inttypes.h>

#include "Protocol.h"
#include "GreatCircle.h"
#include "AVRLinkedList.h"
#include "NMEA.h"

#ifndef DRONEUTILS_H
#define DRONEUTILS_H
const static uint8_t GPS_SETUP[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01, 0xFF, 0x18,
						0xB5, 0x62, 0x06, 0x17, 0x04, 0x00, 0x00, 0x23, 0x00, 0x00, 0x44, 0x52,
						0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x8A};

class CommManager{
	HardwareSerial *stream;
	uint8_t buf[Protocol::MAX_MESSAGE_SIZE+2];
	uint8_t bufPos;
	boolean isLooped;
	AVRLinkedList<Point> waypoints;
	unsigned int targetIndex;
public:
	CommManager(HardwareSerial *inStream);
	void update();
	Point getWaypoint(int index);
	int numWaypoints();
	bool loopWaypoints();
	void setTargetIndex(unsigned int index);
	void advanceTargetIndex();
	void retardTargetIndex();
	unsigned int getTargetIndex();
	Point getTargetWaypoint();
	void sendDataMessage(uint8_t tag, double data);
	void sendDataMessage(uint8_t tag, long data);
	void requestResync();
private:
	Point cachedTarget;
	void sendTargetIndex();
	void processMessage(uint8_t* data, uint8_t length);
	void sendConfirm(uint16_t data, HardwareSerial *stream);
	void recieveWaypoint(uint8_t tag, double lat, double lon, uint8_t index);
	void recieveCommand(uint8_t command);
};
#endif
