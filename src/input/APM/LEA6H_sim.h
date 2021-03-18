#ifndef LEA6H_SIM_H
#define LEA6H_SIM_H

#include "Arduino.h"
#include <inttypes.h>

#include "input/Sensor.h"
#include "input/GPS.h"

class LEA6H_sim : public Sensor, public GPS {
protected:

    void calcChecksum(const uint8_t* msg, uint8_t len,
                      uint8_t &c_a, uint8_t &c_b);

    bool m_warning;
    bool m_updatedRMC;
    float m_course;
    float m_dateOfFix;
    float m_groundSpeed;
    double m_latitude;
    double m_longitude;
    float m_magVar;
    float m_timeOfFix;

    //Waypoint m_location;

    uint16_t m_dataIndex;

    //speed?
    //target location?
    //amount of noise in gps location?
    

public:
    LEA6H_sim();
    void begin();
    void end() {}
    Sensor::Status status();
    void calibrate();
    void update();
    
    bool  getWarning()                {  return m_warning;      }
    bool  getUpdatedRMC()             {  return m_updatedRMC;   }    
    void  clearUpdatedRMC()           { m_updatedRMC=false;     }
    void  setUpdatedRMC()             { m_updatedRMC=true;      }
    uint16_t dataIndex()              {  return m_dataIndex;    }
    float getCourse()                 {  return m_course;       }
    void setCourse(float course)      {  m_course = course;     }
    float getDateOfFix()              {  return m_dateOfFix;    }
    float getGroundSpeed()            {  return m_groundSpeed;  }
    void setGroundSpeed(float speed)   {  m_groundSpeed=speed;   }
    double getLatitude()               {  return m_latitude;     }
    void setLatitude(double lat)      {  m_latitude = lat;      }
    double getLongitude()              {  return m_longitude;    }
    void setLongitude(double lng)     {  m_longitude = lng;     }
    float getMagVar()                 {  return m_magVar;       }
    float getTimeOfFix()              {  return m_timeOfFix;    }
    
    Waypoint getLocation()            {  return Waypoint(m_latitude,m_longitude);    }

    //todo
    //void handleCommand();
    
};


Sensor::Status LEA6H_sim::status()
{
  return Sensor::OK;
}

LEA6H_sim::LEA6H_sim()
{
  //adjust location heading to target by some X amount?
  m_warning = false;
  m_updatedRMC=true;
  m_course=90.0;
  m_dateOfFix=100620;
  m_groundSpeed=0.0;
  m_latitude=47.626025;
  m_longitude=-117.644087;
  m_magVar=1.0;
  m_timeOfFix=083559.00; 
}

void LEA6H_sim::begin()
{
 
}

void LEA6H_sim::calibrate()
{

}

void LEA6H_sim::update()
{
    //todo 

  

}




#endif
