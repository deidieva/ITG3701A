#include <Arduino.h>
#include <Wire.h>
#include "ITG3701.h"

enum Gscale {
  GFS_500DPS = 0x00,
  GFS_1000DPS = 0x01,
  GFS_2000DPS =0x10,
  GFS_4000DPS =0x11,
};
Gscale DPS= GFS_500DPS; //select a rate DATAperSecond option from enum Gscale


/*enum Godr {  // set of allowable gyro sample rates
  GODR_95Hz = 0,
  GODR_190Hz,
  GODR_380Hz,
  GODR_760Hz
};*/

/**********************************************************************/
/*Function: Read a byte with the register address of ITG3701.         */
/*Parameter:-uint8_t _register,the register address  of ITG3701 to read; */
/*Return:	-int8_t,the byte that is read from the register.		  */
int8_t ITG3701::read(uint8_t _register)
{
    int8_t data;
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(_register);
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDRESS, 1);
    if(Wire.available() > 0)
    {
        data = Wire.read();
    }
	Wire.endTransmission();
    return data;
}
/*Function: Write a byte to the register of the MMA7660*/
void ITG3701::write(uint8_t _register, uint8_t _data)
{
	Wire.begin();
	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.write(_register);
	Wire.write(_data);
	Wire.endTransmission();
}

/**********************************************************************/
/*Function: Initialization for ITG3200.         					  */
void ITG3701::init()
{
	Wire.begin();
	write(ITG3701_PWR_M,0x00);//send a reset to the device
 	write(ITG3701_SMPL,0x00);//sample rate divider
 	write(ITG3701_DLPF,DPS);//+/-2000 degrees/s (default value)
}

int16_t ITG3701::read(uint8_t addressh, uint8_t addressl)
{
    int data, t_data;

    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(addressh);
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDRESS, 1);
    if(Wire.available() > 0)
    {
        t_data = Wire.read();
        data = t_data << 8;
    }
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(addressl);
    Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDRESS, 1);
    if(Wire.available() > 0)
    {
        data |= Wire.read();
    }
    return data;
}
/*Function: Get the temperature from ITG that with a on-chip*/
/*           temperature sensor.                                */
double ITG3701::getTemperature()
{
	int temp;
	double temperature;
	temp = read(ITG3701_TMP_H, ITG3701_TMP_L);
	temperature = 35+ ((double) (temp + 13200)) / 280;
	return(temperature);
}
/*Function: Get the contents of the registers in the ITG3200*/
/*          so as to calculate the angular velocity.        */
void ITG3701::getXYZ(int16_t *x,int16_t *y,int16_t *z)
{
	*x = read(ITG3701_GX_H,ITG3701_GX_L)+x_offset;
	*y = read(ITG3701_GY_H,ITG3701_GY_L)+y_offset;
	*z = read(ITG3701_GZ_H,ITG3701_GZ_L)+z_offset;
}
/*Function: Get the angular velocity and its unit is degree per second.*/
void ITG3701::getAngularVelocity(float *ax,float *ay,float *az)
{
  int16_t x,y,z;
	getXYZ(&x,&y,&z);

 if(DPS=0x00){
  int16_t XerrVal0=-10.29;
   int16_t YerrVal0=29.56;
   int16_t ZerrVal0=29.51;
  	*ax = x/65.5-XerrVal0;
  	*ay = y/65.5-YerrVal0;
  	*az = z/65.5-ZerrVal0;
  }
else if(DPS=0x01){
  int16_t XerrVal1=-11.36;
   int16_t YerrVal1=28.64;
   int16_t ZerrVal1=20.83;
    *ax = x/32.8-XerrVal1;
    *ay = y/32.8-YerrVal1;
    *az = z/32.8-ZerrVal1;

  }
else if(DPS=0x10) {
  int16_t XerrVal2=11.07;
   int16_t YerrVal2=-18.06;
   int16_t ZerrVal2=-10.10;
    *ax = x/16.4-XerrVal2;
    *ay = y/16.4-YerrVal2;
    *az = z/16.4-ZerrVal2;
  }
else if (DPS=0x11){
  int16_t XerrVal3=10.56;
   int16_t YerrVal3=-18.38;
   int16_t ZerrVal3=-10.04;
    *ax = x/8.2-XerrVal3;
  	*ay = y/8.2-YerrVal3;
  	*az = z/8.2-ZerrVal3;
  }
  else {
    *ax = x;
  	*ay = y;
  	*az = z;
  }

}
/////////////////////////////////////////////

void ITG3701::zeroCalibrate(unsigned int samples, unsigned int sampleDelayMS)
{
  int16_t x_offset_temp = 0;
  int16_t y_offset_temp = 0;
  int16_t z_offset_temp = 0;
  int16_t x,y,z;
  x_offset = 0;
  y_offset = 0;
  z_offset = 0;
  getXYZ(&x,&y,&z);//
  for (int i = 0;i < samples;i++){
    delay(sampleDelayMS);
    getXYZ(&x,&y,&z);
    x_offset_temp += x;
    y_offset_temp += y;
    z_offset_temp += z;
  }

  x_offset = abs(x_offset_temp)/samples;
  y_offset = abs(y_offset_temp)/samples;
  z_offset = abs(z_offset_temp)/samples;
  if(x_offset_temp > 0)x_offset = -x_offset;
  if(y_offset_temp > 0)y_offset = -y_offset;
  if(z_offset_temp > 0)z_offset = -z_offset;

}
