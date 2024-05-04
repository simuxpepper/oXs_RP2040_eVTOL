/*
  XGZP6897D.h - Library for using a familly of pressure sensors from CFSensor.com
  I2C sensors
  Created by Francis Sourbier
  GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
  This library is free software;
  Released into the public domain
*/
#include "stdint.h"
//#ifndef TEENSY_H
//#define TEENSY_H

//#define debugFS
#define Teensy_device_address 0x6C
class TEENSY
{
  public:
    TEENSY( uint8_t numParams); // K depends on sensor. See datasheet.
    void begin();  // true: device responding.  false:device not responding
    bool fcInstalled = false;

    int16_t rollT, pitchT, yawT;
    float throt, ail, ele, rud, aux1, aux2;

    // readSensor:
    //  Read temperature (degree Celsius), and pressure (PA)
    //  Return float values float &temperature, float &pressure
    void readSensor();
    //  readRawSensor:
    //  Return raw integer values for temperature and pressure.
    //  The raw integer value of temperature must be devided by 256 to convert in degree Celsius.
    //  The raw integer value of pressure must be devided by the K factor to convert in Pa.
    //void readRawSensor(int16_t &rawTemperature, int32_t &rawPressure);
    //void getDifPressure();
  private:
    uint8_t _nParams;
    uint8_t _address;
    
    //uint8_t readBuffer[10];                 // get 10 bytes returned by Teensy
    uint32_t prevReadUs ;
};

//#endif