#ifndef LEA6H_H
#define LEA6H_H

#include "Arduino.h"
#include <inttypes.h>
#include "comms/NMEA.h"
#include "input/Sensor.h"
#include "input/GPS.h"
#include "math/floatgps.h"

class LEA6H : public Sensor, public GPS {
protected:
    //GPS_SETUP messages
    const static uint8_t GPS_RESET[];
    //const static uint8_t GPS_SETUP[];
    const static uint8_t Pedestrian_Mode[];
    const static uint8_t GPRMC_On[];
    const static uint8_t GPGNS_On[];
    const static uint8_t GPGGA_Off[];
    const static uint8_t GPGSA_Off[];
    const static uint8_t GPGSV_Off[];
    const static uint8_t GPVTG_Off[];
    const static uint8_t GNGLL_Off[];


    const static uint8_t CFG_NMEA[];
    const static uint8_t CFG_PRT[];
    const static uint8_t CFG_RATE[];
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
    bool  getWarning()      { update(); return parser.getWarning();     }
    bool  getUpdatedRMC()   { update(); return parser.getUpdatedRMC();  }    
    void  clearUpdatedRMC() { parser.clearUpdatedRMC();                 }
    uint16_t dataIndex()    { update(); return parser.dataIndex();      }
    float getCourse()       { update(); return parser.getCourse();      }
    float getDateOfFix()    { update(); return parser.getDateOfFix();   }
    float getGroundSpeed()  { update(); return parser.getGroundSpeed(); }

    GPS_COORD getGPS_COORD() { update(); return parser.getGPS_COORD();  }

    float getLatitude()     { update(); return parser.getLatitude();    }
    float getLongitude()    { update(); return parser.getLongitude();   }

    float getMagVar()       { update(); return parser.getMagVar();      }
    float getTimeOfFix()    { update(); return parser.getTimeOfFix();   }
    uint16_t getNumSat()    { update(); return parser.getNumSat();      }
    float getHDOP()         { update(); return parser.getHDOP();        }
    #ifdef WAYPOINT_H
    Waypoint getLocation()  { update(); return parser.getLocation();    }
    #endif
};
Sensor::Status
LEA6H::status(){
  return Sensor::OK;
}
void
LEA6H::begin()
{
    //reset gps.  Both baud rates starting with fastest
    stream.begin(38400);
    sendUBloxMessage(0x06,0x04, 0x0004,GPS_RESET);
    delay(1000);
    stream.begin(9600);
    sendUBloxMessage(0x06,0x04, 0x0004,GPS_RESET);
    delay(1000);
    //stream.begin(9600);
    sendUBloxMessage(0x06, 0x00, 0x0014, CFG_PRT);

    //Looks like delay needed when switching baud rates
    delay(100);
    stream.flush();
    stream.begin(38400);

    //Turn on RMC and GNS
    sendUBloxMessage(0x06, 0x01, 0x0003, GPRMC_On);
    sendUBloxMessage(0x06, 0x01, 0x0003, GPGNS_On);

    //Turn off all others
    sendUBloxMessage(0x06, 0x01, 0x0003, GPGGA_Off);
    sendUBloxMessage(0x06, 0x01, 0x0003, GPGSA_Off);
    sendUBloxMessage(0x06, 0x01, 0x0003, GPGSV_Off);
    sendUBloxMessage(0x06, 0x01, 0x0003, GPVTG_Off);
    sendUBloxMessage(0x06, 0x01, 0x0003, GNGLL_Off);


    sendUBloxMessage(0x06, 0x17, 0x0004, CFG_NMEA);
    sendUBloxMessage(0x06, 0x24, 0x0024, Pedestrian_Mode);
    sendUBloxMessage(0x06, 0x08, 0x0006, CFG_RATE);

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

const uint8_t LEA6H::GPS_RESET[] = {
    0x00, 0x00, 0x01, 0x00 
};

// const uint8_t LEA6H::GPS_SETUP[] = {
//                         0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01, 0xFF, 0x18,
//                         0xB5, 0x62, 0x06, 0x17, 0x04, 0x00, 0x00, 0x23, 0x00, 0x00, 0x44, 0x52,
//                         0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08,
//                         0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x8A};
const uint8_t LEA6H::Pedestrian_Mode[] = {0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00,
                                          0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                                          0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
                                          0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint8_t LEA6H::GPRMC_On[] = { 0xF0, 0x04, 0x01 };
const uint8_t LEA6H::GPGNS_On[] = { 0xF0, 0x0D, 0x01 };
const uint8_t LEA6H::GPGGA_Off[] = { 0xF0, 0x00, 0x00 };
const uint8_t LEA6H::GPGSA_Off[] = { 0xF0, 0x02, 0x00 };
const uint8_t LEA6H::GPGSV_Off[] = { 0xF0, 0x03, 0x00 };
const uint8_t LEA6H::GPVTG_Off[] = { 0xF0, 0x05, 0x00 };
const uint8_t LEA6H::GNGLL_Off[] = { 0xF0, 0x01, 0x00 };


const uint8_t LEA6H::CFG_NMEA[] = { 0x00, 0x23, 0x00, 0x00 };
const uint8_t LEA6H::CFG_PRT[] =  { 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08,
                                    0x00, 0x00, 0x00, 0x96, 0x00, 0x00,
                                    0x07, 0x00, 0x02, 0x00, 0x00, 0x00,
                                    0x00, 0x00};
const uint8_t LEA6H::CFG_RATE[] = { 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00 };
#endif
