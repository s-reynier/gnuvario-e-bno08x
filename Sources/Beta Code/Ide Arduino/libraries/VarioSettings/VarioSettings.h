#ifndef _VARIO_SETTINGS_H_
#define _VARIO_SETTINGS_H_

#include <Arduino.h>

#include "DebugConfig.h"
#include <HardwareConfig.h>

#ifdef HAVE_SDCARD
#include <sdcardHAL.h>
#endif

#include <ArduinoJson.h>

#define PARAMS_VERSION "1.7"

/*----------------------------*/
/*          DEFAULT           */
/*      Vario parameters      */
/*                            */
/*----------------------------*/

#define DEFAULT_VARIOMETER_PILOT_NAME  										"Magali"
#define DEFAULT_VARIOMETER_GLIDER_NAME 										"MAC-PARA Muse 3"
#define DEFAULT_VARIOMETER_GLIDER_SELECT									0
#define DEFAULT_VARIOMETER_TIME_ZONE  										(+2) 
#define DEFAULT_VARIOMETER_BASE_PAGE_DURATION 						3000
#define DEFAULT_VARIOMETER_MULTIDISPLAY_DURATION 			  	2000
#define DEFAULT_VARIOMETER_BEEP_VOLUME 										3
#define DEFAULT_VARIOMETER_SINKING_THRESHOLD 							-2.0
#define DEFAULT_VARIOMETER_CLIMBING_THRESHOLD							0.2
#define DEFAULT_VARIOMETER_NEAR_CLIMBING_SENSITIVITY			0.5
#define DEFAULT_VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM   	false
#define DEFAULT_VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP    	false
#define DEFAULT_FLIGHT_START_MIN_TIMESTAMP 								15000
#define DEFAULT_FLIGHT_START_VARIO_LOW_THRESHOLD 					(-0.5)
#define DEFAULT_FLIGHT_START_VARIO_HIGH_THRESHOLD 				0.5
#define DEFAULT_FLIGHT_START_MIN_SPEED 										8.0
#define DEFAULT_VARIOMETER_RECORD_WHEN_FLIGHT_START 			true
#define DEFAULT_ALARM_SDCARD 															true
#define DEFAULT_ALARM_GPSFIX 															true
#define DEFAULT_ALARM_FLYBEGIN 														true
#define DEFAULT_SETTINGS_CLIMB_PERIOD_COUNT  							10
#define DEFAULT_SETTINGS_GLIDE_RATIO_PERIOD_COUNT 				20
#define DEFAULT_VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE	false
#define DEFAULT_RATIO_CLIMB_RATE 													1
#define DEFAULT_NO_RECORD 																false
#define DEFAULT_BATTERY_TONE_FREQHZ												400
#define DEFAULT_CALIB_TONE_FREQHZ													800
#define DEFAULT_MPU9250_ERROR_TONE_FREQHZ									200
#define DEFAULT_MS5611_ERROR_TONE_FREQHZ									2500
#define DEFAULT_SDCARD_ERROR_TONE_FREQHZ									2000
#define DEFAULT_BEEP_FREQ                  								800

#define LK8000_SENTENCE																		0
#define LXNAV_SENTENCE																		1

#define DEFAULT_VARIOMETER_SENT_LXNAV_SENTENCE						1

#define DEFAULT_BLUETOOTH_SEND_CALIBRATED_ALTITUDE        false

#define DEFAULT_VARIOMETER_SSID_1													"your_SSID1"
#define DEFAULT_VARIOMETER_PASSWORD_1											"your_PASSWORD_for SSID1"

#define DEFAULT_VARIOMETER_SSID_2													"your_SSID2"
#define DEFAULT_VARIOMETER_PASSWORD_2											"your_PASSWORD_for SSID2"

#define DEFAULT_VARIOMETER_SSID_3													"your_SSID3"
#define DEFAULT_VARIOMETER_PASSWORD_3											"your_PASSWORD_for SSID3"

#define DEFAULT_VARIOMETER_SSID_4													"your_SSID4"
#define DEFAULT_VARIOMETER_PASSWORD_4											"your_PASSWORD_for SSID4"

#define DEFAULT_URL_UPDATE 																"http://gnuvario-e.yj.fr/webupdate/checkversion"

#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_00								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_01								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_02								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_03								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_04								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_05								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_06								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_07								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_08								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_09								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_10								0x00
#define DEFAULT_VERTACCEL_GYRO_CAL_BIAS_11								0x00
#define DEFAULT_VERTACCEL_ACCEL_CAL_BIAS_00								0x00
#define DEFAULT_VERTACCEL_ACCEL_CAL_BIAS_01								0x00
#define DEFAULT_VERTACCEL_ACCEL_CAL_BIAS_02								0x00
#define DEFAULT_VERTACCEL_ACCEL_CAL_SCALE 								0
#define DEFAULT_VERTACCEL_MAG_CAL_BIAS_00									0
#define DEFAULT_VERTACCEL_MAG_CAL_BIAS_01									0
#define DEFAULT_VERTACCEL_MAG_CAL_BIAS_02									0
#define DEFAULT_VERTACCEL_MAG_CAL_PROJ_SCALE 							-16689
#define DEFAULT_VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER 			7
#define DEFAULT_VERTACCEL_MAG_CAL_BIAS_MULTIPLIER 				5

#define DEFAULT_RATIO_MAX_VALUE														30.0
#define DEFAULT_RATIO_MIN_SPEED														10.0

#define DEFAULT_VARIOMETER_ENABLE_BT										  0
#define DEFAULT_ALARM_VARIOBEGIN                          1

#define DEFAULT_COMPENSATION_TEMP													-6
#define DEFAULT_COMPENSATION_GPSALTI											-50

#define DEFAULT_SLEEP_TIMEOUT_MINUTES 										20
#define DEFAULT_SLEEP_THRESHOLD_CPS												0.5
#define DEFAULT_ALTERNATE_DATA_DURATION										2000

#define DEFAULT_DISPLAY_STAT_DURATION                     6

#define DEFAULT_VARIOMETER_ENABLE_AGL											true

#ifdef HAVE_ACCELEROMETER
#define DEFAULT_ACCELERATION_MEASURE_STANDARD_DEVIATION 	0.35   //0.30
#else
#define DEFAULT_ACCELERATION_MEASURE_STANDARD_DEVIATION 	0.6
#endif //HAVE_ACCELEROMETER

#define DEFAULT_LANGUAGE																	0

#define DEFAULT_VARIOMETER_INTEGRATED_CLIMB_RATE					false
#define DEFAULT_SETTINGS_VARIO_PERIOD_COUNT  							5

#define DEFAULT_REF_VOLTAGE       							          2280

#define DEFAULT_MUTE_VARIOBEGIN                           false
  
/*----------------------------*/
/*          SOFTWARE          */
/*      Vario parameters      */
/*                            */
/*----------------------------*/

#define VARIOMETER_MODEL "GNUVarioE"
#define VARIOMETER_MODEL_NAME "GnuVario-E"

/********************/
/* Measure behavior */
/********************/

/* Speed filtering :                                               */
/* Greater values give smoother speed. The base unit is 2 seconds  */
/* so size = 5 use the last 10 seconds to average speed.           */
#define VARIOMETER_SPEED_FILTER_SIZE 5

/**********************/
/* alti/vario objects */
/**********************/
#define POSITION_MEASURE_STANDARD_DEVIATION 0.1

/**********************/
/* Object screen      */
/**********************/

#define SCREEN_MODEL_154 "154"
#define SCREEN_MODEL_290 "290"
#define SCREEN_MODEL_213 "213"

/******************************************************/
/******************************************************/

class VarioSettings {

 public:
  boolean initSettings(bool initSD);
#ifdef HAVE_SDCARD
  boolean readSDSettings(char *FileName, boolean *ModifiedValue);
  boolean readFlashSDSettings();
  void writeFlashSDSettings();
	void loadConfigurationVario(char *filename);
	void saveConfigurationVario(char *filename);
	void writeWifiSDSettings(char *filename);
	
	void setVersion(uint8_t version, uint8_t subVersion, uint8_t betaVersion);
	String getVersion(void);
	String getScreenModel(void);
	
#endif
  uint8_t soundSettingRead(void);
  void soundSettingWrite(uint8_t volume);

#ifdef SDCARD_DEBUG 
  int exINT = 15;
  float exFloat = 1.12345;
  boolean exBoolean = true;
  long exLong = 2123456789;
#endif //SDCARD_DEBUG
  
  String VARIOMETER_PILOT_NAME 	= DEFAULT_VARIOMETER_PILOT_NAME;
	String VARIOMETER_GLIDER_TAB[4] = {DEFAULT_VARIOMETER_GLIDER_NAME, "", "", ""};
	uint8_t VARIOMETER_GLIDER_SELECT = DEFAULT_VARIOMETER_GLIDER_SELECT;
  String VARIOMETER_GLIDER_NAME = DEFAULT_VARIOMETER_GLIDER_NAME;
  
  /* time zone relative to UTC */
  int8_t VARIOMETER_TIME_ZONE 	= DEFAULT_VARIOMETER_TIME_ZONE; 

  /*******************/
/* Screen behavior */
/*******************/

/* the duration of the two screen pages in milliseconds */
  int16_t VARIOMETER_BASE_PAGE_DURATION 				= DEFAULT_VARIOMETER_BASE_PAGE_DURATION;
	int16_t VARIOMETER_MULTIDISPLAY_DURATION 			= DEFAULT_VARIOMETER_MULTIDISPLAY_DURATION;

  /*********/
  /* Beeps */
  /*********/

  /* The volume of the beeps, max = 10 */
  uint8_t VARIOMETER_BEEP_VOLUME								= DEFAULT_VARIOMETER_BEEP_VOLUME;

  /* The variometer react like this according to vertical speed in m/s :        */
  /* (near climbing beep is not enabled by default)                             */
  /*                                                                            */
  /* <--LOW-BEEP--|------SILENT------|--NEAR-CLIMBING-BEEP--|--CLIMBING-BEEP--> */
  /*              |                  |                      |                   */
  /*           SINKING         CLIMBING-SENSITIVITY      CLIMBING               */
  float VARIOMETER_SINKING_THRESHOLD 						= DEFAULT_VARIOMETER_SINKING_THRESHOLD;
  float VARIOMETER_CLIMBING_THRESHOLD						=	DEFAULT_VARIOMETER_CLIMBING_THRESHOLD;
  float VARIOMETER_NEAR_CLIMBING_SENSITIVITY		= DEFAULT_VARIOMETER_NEAR_CLIMBING_SENSITIVITY; 
  
  /* The near climbing alarm : signal that you enter or exit the near climbing zone */
  /* The near climbing beep : beep when you are in near climbing zone               */
  boolean VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM = DEFAULT_VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM;
  boolean VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP  = DEFAULT_VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP;

  /********************/
  /* Measure behavior */
  /********************/

  /* Flight start detection conditions :                      */
  /* -> Minimum time after poweron in milliseconds            */
  /* -> Minimum vertical velocity in m/s (low/high threshold) */
  /* -> Minimum ground speed in km/h                          */
  long FLIGHT_START_MIN_TIMESTAMP 							= DEFAULT_FLIGHT_START_MIN_TIMESTAMP;
  float FLIGHT_START_VARIO_LOW_THRESHOLD 				= DEFAULT_FLIGHT_START_VARIO_LOW_THRESHOLD;
  float FLIGHT_START_VARIO_HIGH_THRESHOLD 			= DEFAULT_FLIGHT_START_VARIO_HIGH_THRESHOLD;
  float FLIGHT_START_MIN_SPEED 									= DEFAULT_FLIGHT_START_MIN_SPEED;

  /* GPS track recording on SD card starting condition :  */ 
  /* -> As soon as possible (GPS fix)                     */
  /* -> When flight start is detected                     */
  boolean VARIOMETER_RECORD_WHEN_FLIGHT_START 	= DEFAULT_VARIOMETER_RECORD_WHEN_FLIGHT_START;

  /* What type of vario NMEA sentence is sent by bluetooth. */
  /* Possible values are :                                  */
  /*  - VARIOMETER_SENT_LXNAV_SENTENCE                      */
  /*  - VARIOMETER_SENT_LK8000_SENTENCE                     */
  int VARIOMETER_SENT_LXNAV_SENTENCE 			      	= DEFAULT_VARIOMETER_SENT_LXNAV_SENTENCE;
	
	bool BLUETOOTH_SEND_CALIBRATED_ALTITUDE         = DEFAULT_BLUETOOTH_SEND_CALIBRATED_ALTITUDE;

  /* Alarm */
  /* Alarm SDCARD not insert */
  boolean ALARM_SDCARD 														= DEFAULT_ALARM_SDCARD;
  /* Nip when GPS Fix */
  boolean ALARM_GPSFIX 														= DEFAULT_ALARM_GPSFIX;
  /* Bip when Fly begin */
  boolean ALARM_FLYBEGIN 													= DEFAULT_ALARM_FLYBEGIN;
	
	/* Bip when vario begin  */
	boolean ALARM_VARIOBEGIN    										= DEFAULT_ALARM_VARIOBEGIN;                     

//*****************************************************************************
//*****************************************************************************

// Kalman filter configuration
  float KF_ZMEAS_VARIANCE  			=   400.0f;
  float KF_ZACCEL_VARIANCE 			=   1000.0f;
  float KF_ACCELBIAS_VARIANCE   = 	1.0f;

// Power-down timeout. Here we power down if the
// vario does not see any climb or sink rate more than
// 50cm/sec, for 20 minutes.
//   uint16_t SLEEP_TIMEOUT_SECONDS = 1200; // 20 minutes
//   uint8_t  SLEEP_THRESHOLD_CPS		= 50;

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet
/*                                                                            */
/* <--LOW-BEEP--|------SILENT------|--NEAR-CLIMBING-BEEP--|--CLIMBING-BEEP--> */
/*              |                  |                      |                   */
/*             SINK              ZERO                   CLIMB                 */
   uint8_t CLIMB_THRESHOLD   			=   50;
   int8_t ZERO_THRESHOLD	 				=    5;
   int16_t SINK_THRESHOLD    			=   -250;

// change these parameters based on the frequency bandwidth of the speaker

    uint16_t VARIO_MAX_FREQHZ   	=   4000;
    uint16_t VARIO_XOVER_FREQHZ 	=   2000;
    uint16_t VARIO_MIN_FREQHZ   	=   200;

    uint16_t VARIO_SINK_FREQHZ  	=   400;
    uint16_t VARIO_TICK_FREQHZ  	=   200;

		float 	 RATIO_MAX_VALUE 			=		DEFAULT_RATIO_MAX_VALUE;
		float		 RATIO_MIN_SPEED 			=		DEFAULT_RATIO_MIN_SPEED;
		
//Setting accelerometer
    double ACCELCALX = 0.0;
		double ACCELCALY = 0.0;
		double ACCELCALZ = 0.0;
		
//********************************************************
//********************************************************		

// audio feedback tones
    uint16_t BATTERY_TONE_FREQHZ												=	DEFAULT_BATTERY_TONE_FREQHZ;
    uint16_t CALIB_TONE_FREQHZ													=	DEFAULT_CALIB_TONE_FREQHZ;
    uint16_t MPU9250_ERROR_TONE_FREQHZ									= DEFAULT_MPU9250_ERROR_TONE_FREQHZ;
    uint16_t MS5611_ERROR_TONE_FREQHZ										= DEFAULT_MS5611_ERROR_TONE_FREQHZ;
    uint16_t SDCARD_ERROR_TONE_FREQHZ										= DEFAULT_SDCARD_ERROR_TONE_FREQHZ;  
		uint16_t BEEP_FREQ                  								= DEFAULT_BEEP_FREQ ;
  
//Setting FightHistory	
//!!!!!!!!!!!!
// need to be moved to settings 
// unit is 500ms
		int8_t SETTINGS_CLIMB_PERIOD_COUNT  								= DEFAULT_SETTINGS_CLIMB_PERIOD_COUNT;
		int8_t SETTINGS_GLIDE_RATIO_PERIOD_COUNT 						= DEFAULT_SETTINGS_GLIDE_RATIO_PERIOD_COUNT;


		boolean VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE 		= DEFAULT_VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE;
		uint8_t RATIO_CLIMB_RATE 														= DEFAULT_RATIO_CLIMB_RATE;
		
		boolean NO_RECORD 																	= DEFAULT_NO_RECORD;
  
		String VARIOMETER_SSID_1 														= DEFAULT_VARIOMETER_SSID_1;
		String VARIOMETER_PASSWORD_1 												= DEFAULT_VARIOMETER_PASSWORD_1;

		String VARIOMETER_SSID_2 														= DEFAULT_VARIOMETER_SSID_2;
		String VARIOMETER_PASSWORD_2 												= DEFAULT_VARIOMETER_PASSWORD_2;

		String VARIOMETER_SSID_3 														= DEFAULT_VARIOMETER_SSID_3;
		String VARIOMETER_PASSWORD_3 												= DEFAULT_VARIOMETER_PASSWORD_3;

		String VARIOMETER_SSID_4 														= DEFAULT_VARIOMETER_SSID_4;
		String VARIOMETER_PASSWORD_4 												= DEFAULT_VARIOMETER_PASSWORD_4;
		
		uint8_t VARIO_VERTACCEL_GYRO_CAL_BIAS_00						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_00;
		uint8_t VARIO_VERTACCEL_GYRO_CAL_BIAS_01						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_01;
		uint8_t VARIO_VERTACCEL_GYRO_CAL_BIAS_02					  = DEFAULT_VERTACCEL_GYRO_CAL_BIAS_02;
		uint8_t VARIO_VERTACCEL_GYRO_CAL_BIAS_03						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_03;
		uint8_t VARIO_VERTACCEL_GYRO_CAL_BIAS_04						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_04;
		uint8_t VARIO_VERTACCEL_GYRO_CAL_BIAS_05						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_05;
		uint8_t VARIO_VERTACCEL_GYRO_CAL_BIAS_06						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_06;
		uint8_t  VARIO_VERTACCEL_GYRO_CAL_BIAS_07						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_07;
		uint8_t  VARIO_VERTACCEL_GYRO_CAL_BIAS_08						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_08;
		uint8_t  VARIO_VERTACCEL_GYRO_CAL_BIAS_09						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_09;
		uint8_t  VARIO_VERTACCEL_GYRO_CAL_BIAS_10						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_10;
		uint8_t  VARIO_VERTACCEL_GYRO_CAL_BIAS_11						= DEFAULT_VERTACCEL_GYRO_CAL_BIAS_11;
		int16_t VARIO_VERTACCEL_ACCEL_CAL_BIAS_00						= DEFAULT_VERTACCEL_ACCEL_CAL_BIAS_00;
		int16_t VARIO_VERTACCEL_ACCEL_CAL_BIAS_01						= DEFAULT_VERTACCEL_ACCEL_CAL_BIAS_01;
		int16_t VARIO_VERTACCEL_ACCEL_CAL_BIAS_02						= DEFAULT_VERTACCEL_ACCEL_CAL_BIAS_02;
		int16_t VARIO_VERTACCEL_ACCEL_CAL_SCALE    					= DEFAULT_VERTACCEL_ACCEL_CAL_SCALE;
		int16_t VARIO_VERTACCEL_MAG_CAL_BIAS_00							= DEFAULT_VERTACCEL_MAG_CAL_BIAS_00;
		int16_t VARIO_VERTACCEL_MAG_CAL_BIAS_01							= DEFAULT_VERTACCEL_MAG_CAL_BIAS_01;
		int16_t VARIO_VERTACCEL_MAG_CAL_BIAS_02							= DEFAULT_VERTACCEL_MAG_CAL_BIAS_02;
		int16_t VARIO_VERTACCEL_MAG_CAL_PROJ_SCALE 					= DEFAULT_VERTACCEL_MAG_CAL_PROJ_SCALE;
		uint16_t VARIO_VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER	= DEFAULT_VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER;
		uint16_t VARIO_VERTACCEL_MAG_CAL_BIAS_MULTIPLIER 	 	= DEFAULT_VERTACCEL_MAG_CAL_BIAS_MULTIPLIER;
		
		boolean  VARIOMETER_ENABLE_BT 											= DEFAULT_VARIOMETER_ENABLE_BT;

		float    COMPENSATION_TEMP													= DEFAULT_COMPENSATION_TEMP;
		int16_t  COMPENSATION_GPSALTI												= DEFAULT_COMPENSATION_GPSALTI;

		uint8_t	 SLEEP_TIMEOUT_MINUTES											= DEFAULT_SLEEP_TIMEOUT_MINUTES;
		float    SLEEP_THRESHOLD_CPS												= DEFAULT_SLEEP_THRESHOLD_CPS;
		uint16_t ALTERNATE_DATA_DURATION										= DEFAULT_ALTERNATE_DATA_DURATION;
		
		uint8_t  DISPLAY_STAT_DURATION											= DEFAULT_DISPLAY_STAT_DURATION;
		
		String   URL_UPDATE																	= DEFAULT_URL_UPDATE;
		
		boolean  VARIOMETER_ENABLE_AGL											= DEFAULT_VARIOMETER_ENABLE_AGL;
		
		float		 ACCELERATION_MEASURE_STANDARD_DEVIATION 	  = DEFAULT_ACCELERATION_MEASURE_STANDARD_DEVIATION;
		
		uint8_t	 LANGUAGE																		= DEFAULT_LANGUAGE;
		
		boolean  VARIOMETER_INTEGRATED_CLIMB_RATE						= DEFAULT_VARIOMETER_INTEGRATED_CLIMB_RATE;
		
		uint8_t	 SETTINGS_VARIO_PERIOD_COUNT	 							= DEFAULT_SETTINGS_VARIO_PERIOD_COUNT;
		
		uint16_t REF_VOLTAGE																= DEFAULT_REF_VOLTAGE;
		
		boolean  MUTE_VARIOBEGIN														= DEFAULT_MUTE_VARIOBEGIN;
	
		StaticJsonDocument<1900> doc;	
 protected:
#ifdef HAVE_SDCARD
#ifdef SDFAT_LIB
		SdFile myFile;
#else //SDFAT_LIB
		File myFile;
#endif //SDFAT_LIB
#endif
//  File myFile2;
//		char FileName[15] = "SETTINGS.TXT";
		char FileFlashName[15] = "FLASH.TXT";
		String GnuvarioVersion;
  
		boolean applySetting(String settingName, String settingValue);
		void applyFlashSetting(String settingName, String settingValue);
		float toFloat(String settingValue);
		long toLong(String settingValue);
		boolean toBoolean(String settingValue);
};

extern VarioSettings GnuSettings;


/*
class Statistic {

 public:
   void setTime(int8_t* timeValue);
   int8_t* getTime(void);
   int8_t* getTime(int8_t* timeValue);
   void setDuration(int8_t* durationValue);
   int8_t* getDuration(void);
   int8_t* getDuration(int8_t* durationValue);
   void setAlti(double alti);
   double getMaxAlti(void);
   double getMinAlti(void);
   void setVario(double vario);
   double getMaxVario(void);
   double getMinVario(void);
   void setSpeed(double speed);
   double getMaxSpeed(void);
   double getMinSpeed(void);
   double getAltiDeco(void);
   double getGain(void);

  private:
    int8_t time[3];
	int8_t duration[3];

    double currentSpeed=0;
    double maxSpeed;
    double minSpeed;
    double currentAlti=0;
    double maxAlti;
    double minAlti;
	double currentVario=0;
	double maxVario;
	double minVario;
	
	double altiDeco;
	double gain;
};
*/

#endif
