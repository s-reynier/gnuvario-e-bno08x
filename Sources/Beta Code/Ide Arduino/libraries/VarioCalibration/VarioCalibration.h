/* VarioCalibration -- 
 *
 * Copyright 2019 Jean-philippe GOI
 * 
 * This file is part of GnuVario-E.
 *
 * ToneHAL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ToneHAL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*********************************************************************************/
/*                                                                               */
/*                           VarioCalibration                                    */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0      29/09/19                                                          */
/*    1.0.1    01/10/19    Ajout detection bouton                                */
/*    1.0.2    28/10/19    Ajout enable ampli avant chaque appelle à tone        */
/*    1.0.3    29/11/19    Modif sdfat                                           */
/*                                                                               */
/*********************************************************************************/


#ifndef VARIOCALIBRATION_H
#define VARIOCALIBRATION_H

#include <Arduino.h>
#include <HardwareConfig.h>
#include <DebugConfig.h>

#include <digit.h>

#ifdef HAVE_SDCARD
#include <sdcardHAL.h>
#endif	//HAVE_SDCARD

#include <VarioSettings.h>


class VarioCalibration {
	
public:
   
  void Begin(void);
  
private:

/********************************/
/* Standard deviation recording */
/********************************/
	unsigned long accelSDRecordTimestamp;
	double rawAccelSD;

	int recordInitState = 0;

	double referencePressure;

/*****************/
/* measures data */
/*****************/
	unsigned long measureTimestamp;

/* accel measures */
	int16_t lastAccelMeasure[3];
	long accelCount;
	double accelMean[3];
	double accelSD[3];


#if defined(SDCARD_OUTPUT) && defined(HAVE_SDCARD)
#ifdef SDFAT_LIB
	SdFile file;
#else //SDFAT_LIB
	File file;
#endif //SDFAT_LIB

	boolean sdcardFound = false;
	char filename[15] = "/RECORD00.CAL";
#define FILENAME_SIZE 8
	Digit valueDigit;

	void writeNumber(int16_t number);
#endif //SDCARD_OUTPUT
	
	void startMeasure(void);
	uint8_t readRawAccel(int16_t* accel, int32_t* quat);

#ifdef AK89xx_SECONDARY
/*******************************/
	uint8_t readRawMag(int16_t* mag);
#endif //AK89xx_SECONDARY

	void makeMeasureStep(void);
	double getAccelMeasure(int16_t* accelMeasure);

#ifdef AK89xx_SECONDARY
/* return standard deviation */
/*******************************/
	double getMagMeasure(int16_t* magMeasure);
#endif //AK89xx_SECONDARY

};

extern VarioCalibration Calibration;
#endif
