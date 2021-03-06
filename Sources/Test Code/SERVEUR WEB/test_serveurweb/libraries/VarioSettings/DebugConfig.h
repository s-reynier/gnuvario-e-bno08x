#ifndef _DEBUGCONFIG_H_
#define _DEGUGCONFIG_H_

//Monitor Port 
#if defined(ESP8266)
#define SerialPort Serial
#elif defined(ESP32)
#define SerialPort Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialPort SerialUSB
#elif defined(_BOARD_GENERIC_STM32F103C_H_)

#elif defined(ARDUINO_AVR_PRO)
#define SerialPort Serial
#else
#define SerialPort Serial
#endif

#define ENABLE_DEBUG

#if defined(ENABLE_DEBUG)
// 							OUTPUT SERIALNMEA
//#define SERIAL_NMEA_SERIAL_OUTPUT
//#define SERIAL_NMEA_BLUETOOTH_OUTPUT


//              DEBUGING MODE
//#define IMU_DEBUG			  //debug IMU
#define PROG_DEBUG			  //debug principal program
//#define I2CDEV_SERIAL_DEBUG   //debug I2Cdev
//#define DEBUG_SERIAL_NMEA_1
//#define SCREEN_DEBUG
//#define GPS_DEBUG
//#define BUTTON_DEBUG
//#define TONEDAC_DEBUG
//#define MS5611_DEBUG
//#define KALMAN_DEBUG
//#define ACCEL_DEBUG
#define EEPROM_DEBUG
//#define NMEAPARSER_DEBUG
//#define VOLTAGE_DIVISOR_DEBUG
//#define SDCARD_DEBUG
//#define IGC_DEBUG
#define DATA_DEBUG

#endif //ENABLE_DEBUG
#endif
