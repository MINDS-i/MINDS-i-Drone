#ifndef LEA6H_H
#define LEA6H_H

#include "Arduino.h"
#include <inttypes.h>
#include "comms/NMEA.h"
#include "input/Sensor.h"

class LEA6H : public Sensor{
protected:
    //GPS_SETUP messages
    const static uint8_t GPS_SETUP[];
    const static uint8_t Pedestrian_Mode[];
    const static uint8_t GPRMC_On[];
    const static uint8_t CFG_NMEA[];
    const static uint8_t CFG_PRT[];
    void sendUBloxMessage(uint8_t Type, uint8_t ID,
                          uint16_t len, const uint8_t* buf);
    void calcChecksum(const uint8_t* msg, uint8_t len,
                      uint8_t &c_a, uint8_t &c_b);
    HardwareSerial &stream;
    NMEA            parser;
public:
    LEA6H(HardwareSerial &port): stream(port), parser(stream) {}
    #if defined(__AVR_ATmega2560__)
    LEA6H(): stream(Serial1), parser(stream) {}
    #endif
    void begin();
    void end() {}
    Sensor::Status status();
    void calibrate();
    void update();
    //expose interface provided by NMEA parser
    bool  getWarning()    { update(); return parser.getWarning();     }
    bool  newData()       { update(); return parser.newData();        }
    float getCourse()     { update(); return parser.getCourse();      }
    float getDateOfFix()  { update(); return parser.getDateOfFix();   }
    float getGroundSpeed(){ update(); return parser.getGroundSpeed(); }
    float getLatitude()   { update(); return parser.getLatitude();    }
    float getLongitude()  { update(); return parser.getLongitude();   }
    float getMagVar()     { update(); return parser.getMagVar();      }
    float getTimeOfFix()  { update(); return parser.getTimeOfFix();   }
    #ifdef WAYPOINT_H
    Waypoint getLocation(){ update(); return parser.getLocation();    }
    #endif
};
Sensor::Status
LEA6H::status(){
  if(parser.getWarning()){
    /*#GPSWARN LEA6H gps warning; GPS is running but does not have a lock */
    return Sensor::BAD("GPSWARN");
  }
  return Sensor::OK;
}
void
LEA6H::begin(){
    stream.begin(38400);
    sendUBloxMessage(0x06, 0x01, 0x0003, GPRMC_On);
    sendUBloxMessage(0x06, 0x17, 0x0004, CFG_NMEA);
    sendUBloxMessage(0x06, 0x00, 0x0014, CFG_PRT);
    sendUBloxMessage(0x06, 0x24, 0x0024, Pedestrian_Mode);
}
void
LEA6H::calibrate(){
}
void
LEA6H::update(){
    parser.update();
}
void
LEA6H::sendUBloxMessage(uint8_t Type, uint8_t ID, uint16_t len, const uint8_t* buf){
  uint8_t header[4];
  uint8_t check[2] = {0,0};
  header[0] = Type;
  header[1] = ID;
  header[2] = (uint8_t) (len&0xff);
  header[3] = (uint8_t) len>>8;

  calcChecksum(header, 4, check[0], check[1]);
  if(len > 0) calcChecksum(buf, len, check[0], check[1]);

  stream.write((char)0xB5);
  stream.write((char)0x62);
  stream.write(header, 4);
  if(len > 0) stream.write(buf, len);
  stream.write(check, 2);
}
void
LEA6H::calcChecksum(const uint8_t *msg, uint8_t len, uint8_t &c_a, uint8_t &c_b){
    while (len--) {
        c_a += *msg;
        c_b += c_a;
        msg++;
    }
}

const uint8_t LEA6H::GPS_SETUP[] = {
                        0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01, 0xFF, 0x18,
                        0xB5, 0x62, 0x06, 0x17, 0x04, 0x00, 0x00, 0x23, 0x00, 0x00, 0x44, 0x52,
                        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08,
                        0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x8A};
const uint8_t LEA6H::Pedestrian_Mode[] = {0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00,
                                          0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                                          0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
                                          0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint8_t LEA6H::GPRMC_On[] = { 0xF0, 0x04, 0x01 };
const uint8_t LEA6H::CFG_NMEA[] = { 0x00, 0x23, 0x00, 0x00 };
const uint8_t LEA6H::CFG_PRT[] =  { 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08,
                                    0x00, 0x00, 0x00, 0x96, 0x00, 0x00,
                                    0x07, 0x00, 0x02, 0x00, 0x00, 0x00,
                                    0x00, 0x00};
#endif
