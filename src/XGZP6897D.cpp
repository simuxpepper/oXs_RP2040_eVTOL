/*
  XGZP6897D.cpp - Library for using a familly of pressure sensors from CFSensor.com
  I2C sensors
  Created by Francis Sourbier
  GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
  This library is free software; 
  Released into the public domain
*/

#include "XGZP6897D.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "param.h"
#include "hardware/watchdog.h"
#include <math.h>

extern CONFIG config;
extern float actualPressurePa; // this value is updated when baro1,2 or 3 is installed
extern float difPressureAirspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureAirspeedCount;
extern float difPressureCompVspeedSumPa; // calculate a moving average on x values
extern uint32_t difPressureCompVspeedCount;
extern float temperatureKelvin;     // in Kelvin , used when airspeed is calculated

//  Descriptor. K depends on the exact model of the sensor. See datasheet and documentation
XGZP6897D::XGZP6897D(uint16_t K)
{
  _K = K;
  _address = I2C_device_address;
}

void XGZP6897D::begin()
{
  airspeedInstalled = false;
  if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
  
  //uint8_t readBuffer[9];
  int32_t i2cError;
  
  // set the sensor in continous mode with averaging (send a command 0X3615)
  uint8_t cmdData[2] = {0X30 , 0X0A} ; 
  if ( i2c_write_timeout_us (i2c1 , _address, &cmdData[0] , 2 , true , 1000) <0 ) {
      printf("error write command to xgzp6897d\n");
      return ; 
  }
  sleep_ms(100);

  i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 5 , true, 5500) ;    
  if (i2cError < 0) {
      printf("read 5 bytes from xgzp6897d = %d \n", (int) i2cError);    
      return ; // skip if there is an error
  }  
  // first 3 bytes = pressure, byte 4 & 5 = temp
  //nextPressureReadMillis = millisRp() + 2;    //
  //nextAirSpeedMillis  = nextPressureReadMillis + 200 ; 
  temperatureCelsius =   ((float)(( (((int16_t)readBuffer[3]) << 8) & 0X7F00)  | readBuffer[4])) / 256.0  ;
  temperatureKelvin = 273.15 + temperatureCelsius ; // in Kelvin
  //dpScaleSdp3x =  966.0 / 1013.25 / ((float)((int16_t)readBuffer[6] << 8 | readBuffer[7]));
  // datasheet says that we have to apply a correction of 966/actual pressure in mbar; it is estimated with 1013

  printf("xgzp6897d has been succesfuly detected\n");
  airspeedInstalled = true;
  prevReadUs = microsRp(); 
}
//  Return raw integer values for temperature and pressure.
//  The raw integer of temperature must be devided by 256 to convert in degree Celsius
//  The raw integer of pressure must be devided by the K factor to convert in Pa
//void XGZP6897D::readRawSensor(int16_t &rawTemperature, int32_t &rawPressure)
void XGZP6897D::getDifPressure()
 {
  if ( ! airspeedInstalled) return ;     // do not process if there is no sensor
    uint32_t now = microsRp();
    if ( (now - prevReadUs) < 2000 ) return ;// it takes about 500 usec for a conversion
    // read a new pressure
    prevReadUs = now;
    if ( i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , 5 , true, 1500) < 0)  {
        printf("error read xgzp6897d (airspeed sensor)\n");
        return;
  }

  difPressurePa =  (float) ((int32_t) ((readBuffer[0] & 0X7F) << 16 | (readBuffer[1] & 0X00FF) << 8 | readBuffer[2] & 0X0000FF)) / _K; // diffPressure in pa
  difPressureAirspeedSumPa += difPressurePa; // calculate a moving average on x values
  difPressureAirspeedCount++;                // count the number of conversion
  difPressureCompVspeedSumPa += difPressurePa; // calculate a moving average on x values
  difPressureCompVspeedCount++;                // count the number of conversion
 }
/*
  int32_t pressure_adc;
  int16_t  temperature_adc ;
  uint8_t pressure_H, pressure_M, pressure_L, temperature_H, temperature_L;
  uint8_t CMD_reg;
  // start conversion
  Wire.beginTransmission(_address);
  Wire.write(0x30);
  Wire.write(0x0A);   //start combined conversion pressure and temperature
  Wire.endTransmission();
  // wait until the end of conversion (Sco bit in CMD_reg. bit 3)
  do {
    Wire.beginTransmission(_address);
    Wire.write(0x30);                       //send 0x30 CMD register address
    Wire.endTransmission();
    Wire.requestFrom(_I2C_address, byte(1));
    CMD_reg = Wire.read();                //read 0x30 register value
  } while ((CMD_reg & 0x08) > 0);        //loop while bit 3 of 0x30 register copy is 1
  // read temperature and pressure
  Wire.beginTransmission(_I2C_address);
  Wire.write(0x06);                        //send pressure high byte register address
  Wire.endTransmission();
  Wire.requestFrom(_I2C_address, byte(5)); // read 3 bytes for pressure and 2 for temperature
  pressure_H = Wire.read();
  pressure_M = Wire.read();
  pressure_L = Wire.read();
  temperature_H = Wire.read();
  temperature_L = Wire.read();
  pressure_adc = ((uint32_t)pressure_H << 8) + (uint32_t) pressure_M;
  pressure_adc = (pressure_adc << 8) + (uint32_t) pressure_L;
  temperature_adc = ((uint16_t)temperature_H << 8) + (uint16_t) temperature_L;
  // pressure is a signed2 complement style value, on 24bits.
  // need to extend the bit sign on the full 32bits.
  rawPressure = ((pressure_adc << 8) >> 8);
  rawTemperature = temperature_adc;
#ifdef debugFS
  Serial.print(String(pressure_H, HEX));
  Serial.print("," + String(pressure_M, HEX));
  Serial.print("," + String(pressure_L, HEX));
  Serial.print(":" + String(pressure_adc, HEX));
  Serial.print(" – " + String(temperature_H, HEX));
  Serial.print("," + String(temperature_L, HEX));
  Serial.print(":" + String(temperature_adc, HEX));
  Serial.println();
#endif
  return ;
} 
//  Read temperature (degree Celsius), and pressure (PA)
//  Return float values
void XGZP6897D::readSensor(float &temperature, float &pressure)
{
  int32_t rawPressure;
  int16_t  rawTemperature ;
  readRawSensor(rawTemperature,rawPressure);
  pressure = rawPressure / _K;
  temperature = float(rawTemperature) / 256;
  #ifdef debugFS
  Serial.print(" - " + String(temperature) + ":" + String(pressure));
  Serial.println();
#endif
}
*/

