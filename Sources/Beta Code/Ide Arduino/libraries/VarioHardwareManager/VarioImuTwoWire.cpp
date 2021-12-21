#include <Arduino.h> 
#include <HardwareConfig.h>
#include <DebugConfig.h>
#include <VarioLog.h>

#include "I2CWrapper.h"
#include "VarioData.h"
#include <MS5611.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include "VarioImuTwoWire.h"

MS5611 ms5611;
BNO080 myIMU;

#include <math.h>

#define R2D 57.2958

//**********************************
VarioImuTwoWire::VarioImuTwoWire()
//**********************************
{
}

//**********************************
void VarioImuTwoWire::init()
//**********************************
{
  // Init BUS I2C  
  I2CWrapper::init();

// Init Moniteur serie  
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  Serial.println(F("ALL CAPTEUR"));

// Init MS5611
  // Initialize MS5611 sensor
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  while(!ms5611.begin(MS5611_ULTRA_HIGH_RES))
  {
    delay(500);
  }

// Init BNO080 sparkfun
  if (myIMU.begin(0x4A, Wire) == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms
  myIMU.enableAccelerometer(50);
  myIMU.enableMagnetometer(50);
}

//**********************************
bool VarioImuTwoWire::havePressure(void)
//**********************************
{
	return true;
}


//**********************************
bool VarioImuTwoWire::updateData(void)
//**********************************
{

	CompteurAccel = 0;
	Temp = ms5611.readTemperature();
    Temp += GnuSettings.COMPENSATION_TEMP; //MPU_COMP_TEMP;
    Accel = myIMU.getAccelY();
		
#ifdef DATA_DEBUG
    SerialPort.print("VarioImuTwoWire Update");
    SerialPort.print("Alti : ");
    SerialPort.println(Alti);
    SerialPort.print("Temperature : ");
    SerialPort.println(Temp);
    SerialPort.print("Accel : ");
    SerialPort.println(Accel);
#endif //DATA_DEBUG
				

		CompteurAccel++;
		if (CompteurAccel > 100) {
			CompteurAccel = 0;    
		}
	
	return true;
}

//**********************************
void VarioImuTwoWire::updateAlti()
//**********************************
{
  long realPressure = ms5611.readPressure();
  Alti = ms5611.getAltitude(realPressure);
}

//**********************************
double VarioImuTwoWire::getAlti()
//**********************************
{
  return Alti; //twScheduler.getAlti();
}

//**********************************
double VarioImuTwoWire::getTemp()
//**********************************
{
  return Temp; //twScheduler.getAlti();
}

//**********************************
double VarioImuTwoWire::getAccel()
//**********************************
{
  return Accel; //twScheduler.getAlti();
}
