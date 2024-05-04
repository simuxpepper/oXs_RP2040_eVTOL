/*
  XGZP6897D.cpp - Library for using a familly of pressure sensors from CFSensor.com
  I2C sensors
  Created by Francis Sourbier
  GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
  This library is free software; 
  Released into the public domain
*/

#include "teensy.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
#include <inttypes.h>
#include "tools.h"
#include "param.h"
#include "hardware/watchdog.h"
#include <math.h>

extern CONFIG config;

//  Descriptor. K depends on the exact model of the sensor. See datasheet and documentation
TEENSY::TEENSY(uint8_t numParams)
{
  _nParams = numParams;
  _address = Teensy_device_address;
}


void TEENSY::begin()
{
  fcInstalled = false;
  if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
  
  //uint8_t readBuffer[9];
  int32_t i2cError;
  //sleep_ms(3000);
  // set the sensor in continous mode with averaging (send a command 0X1010)
  uint8_t cmdData[1] = {0X10} ; 
  //sleep_ms(3000);
  if ( i2c_write_timeout_us (i2c1 , _address, &cmdData[0] , 1 , true , 1000) <0 ) {
      printf("error write command to teensy\n");
      return ; 
  }
  sleep_ms(100);

  uint8_t readBuffer[_nParams * 2];

  i2cError =  i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , _nParams , true, 5500) ;    
  if (i2cError < 0) {
      printf("read %d bytes from teensy = %d \n", (int) _nParams * 2, (int) i2cError);    
      return ; // skip if there is an error
  }  
  // first 3 bytes = pressure, byte 4 & 5 = temp
  //nextPressureReadMillis = millisRp() + 2;    //
  //nextAirSpeedMillis  = nextPressureReadMillis + 200 ; 
  
  //temperatureCelsius =   ((float)(( (((int16_t)readBuffer[3]) << 8) & 0X7F00)  | readBuffer[4])) / 256.0  ;
  //temperatureKelvin = 273.15 + temperatureCelsius ; // in Kelvin
  
  //dpScaleSdp3x =  966.0 / 1013.25 / ((float)((int16_t)readBuffer[6] << 8 | readBuffer[7]));
  // datasheet says that we have to apply a correction of 966/actual pressure in mbar; it is estimated with 1013

  printf("Flight Controller Teensy has been succesfuly detected\n");
  fcInstalled = true;
  prevReadUs = microsRp(); 
}
//  Return raw integer values for temperature and pressure.
//  The raw integer of temperature must be devided by 256 to convert in degree Celsius
//  The raw integer of pressure must be devided by the K factor to convert in Pa
//void XGZP6897D::readRawSensor(int16_t &rawTemperature, int32_t &rawPressure)
void TEENSY::readSensor()
 {
  if ( ! fcInstalled) return ;     // do not process if there is no sensor
  uint32_t now = microsRp();
  if ( (now - prevReadUs) < 5000 ) return ;// it takes about 500 usec for a conversion
  // read a new pressure
  prevReadUs = now;

  uint8_t readBuffer[_nParams * 2];

  uint8_t cmdData[1] = {0X10} ;
  if ( i2c_write_timeout_us (i2c1 , _address, &cmdData[0] , 1 , true , 1000) <0 ) {
    printf("error write command to teensy\n");
    return ; 
  }
  if ( i2c_read_timeout_us (i2c1 , _address , &readBuffer[0] , _nParams * 2 , true, 1500) < 0)  {
      printf("error read teensy (flight controller)\n");
      return;
  }


  rollT = ((int16_t) ((readBuffer[0] << 8) & 0xFF00) | (readBuffer[1] & 0x00FF));
  pitchT = ((int16_t) ((readBuffer[2] << 8) & 0xFF00) | (readBuffer[3] & 0x00FF));
  yawT = ((int16_t) ((readBuffer[4] << 8) & 0xFF00) | (readBuffer[5] & 0x00FF));

  throt = ((uint16_t) ((readBuffer[6] << 8) & 0xFF00) | (readBuffer[7] & 0x00FF));
  ail = ((uint16_t) ((readBuffer[8] << 8) & 0xFF00) | (readBuffer[9] & 0x00FF));
  ele = ((uint16_t) ((readBuffer[10] << 8) & 0xFF00) | (readBuffer[11] & 0x00FF));
  rud = ((uint16_t) ((readBuffer[12] << 8) & 0xFF00) | (readBuffer[13] & 0x00FF));
  aux1 = ((uint16_t) ((readBuffer[14] << 8) & 0xFF00) | (readBuffer[15] & 0x00FF));
  aux2 = ((uint16_t) ((readBuffer[16] << 8) & 0xFF00) | (readBuffer[17] & 0x00FF));

  sent2Core0( PITCH , (int32_t) pitchT ) ; 
  sent2Core0( ROLL , (int32_t) rollT ) ; 
  sent2Core0( YAW , (int32_t) yawT );

  sent2Core0( THROTTLE , (uint16_t) throt ) ; 
  sent2Core0( AILERON , (uint16_t) ail ) ; 
  sent2Core0( ELEVATOR , (uint16_t) ele );
  sent2Core0( RUDDER , (uint16_t) rud ) ; 
  sent2Core0( AUX_1 , (uint16_t) aux1 ) ; 
  sent2Core0( AUX_2 , (uint16_t) aux2 );

  //throt, ail, elev, rud, aux1, aux2

  // throt = (float) ((uint8_t) (readBuffer[3] & 0XFF));
  // ail = (float) ((uint8_t) (readBuffer[4] & 0XFF));
  // ailT = (float) ((uint8_t) (readBuffer[5] & 0XFF));
  // ele = (float) ((uint8_t) (readBuffer[6] & 0XFF));
  // eleT = (float) ((uint8_t) (readBuffer[7] & 0XFF));
  // rud = (float) ((uint8_t) (readBuffer[8] & 0XFF));
  // rudT = (float) ((uint8_t) (readBuffer[9] & 0XFF));

  // printf("Roll: %d, ", (int16_t) rollT);
  // printf("Pitch: %d, ", (int16_t) pitchT);
  // printf("Yaw: %d, ", (int16_t) yawT);

  // printf("Thr: %d, ", (uint16_t) throt);
  // printf("Ail: %d, ", (uint16_t) ail);
  // printf("Ele: %d, ", (uint16_t) ele);
  // printf("Rud: %d, ", (uint16_t) rud);
  // printf("Aux1: %d, ", (uint16_t) aux1);
  // printf("Aux2: %d\n", (uint16_t) aux2);

  // printf("Throttle: %.2f\n", throt);
  // printf("ail: %.2f\n", ail);
  // printf("ailT: %.2f\n", ailT);
  // printf("ele: %.2f\n", ele);
  // printf("eleT: %.2f\n", eleT);
  // printf("rud: %.2f\n", rud);
  // printf("rudT: %.2f\n", rudT);
  //printf("\n");
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

