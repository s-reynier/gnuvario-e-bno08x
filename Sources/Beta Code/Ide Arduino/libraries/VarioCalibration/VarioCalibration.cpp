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
/*  version    Date        Description                                           */
/*    1.0      29/09/19                                                          */
/*    1.0.1    01/10/19    Ajout detection bouton                                */
/*    1.0.2    28/10/19    Ajout enable ampli avant chaque appelle à tone        */
/*    1.0.3    29/11/19    Modif sdfat                                           */
/*                                                                               */
/*********************************************************************************/
#include <Arduino.h>
#include <HardwareConfig.h>
#include <DebugConfig.h>
#include <VarioCalibration.h>
#include <SPI.h>
#include <eepromHAL.h>

#include <VarioSettings.h>


#include <toneHAL.h>
#include <beeper.h>
#include <digit.h>

#ifdef HAVE_SDCARD
#include <sdcardHAL.h>
#endif //HAVE_SDCARD

#include <VarioButton.h>

/* need beeps */
#define MAKE_BEEP
#define BEEP_DURATION 300
#define BEEP_VOLUME 10
#define BEEP_START_FREQ 500
#define BEEP_RECORD_FREQ 1000

/* to compute the standard deviation of the accelerometer */
#define ACCEL_SD_WAIT_DURATION 2000
#define ACCEL_SD_MEASURE_DURATION 8000

/* to make averaged measure */
#define MEASURE_DURATION 500
#define STABILIZE_DURATION 500

/* movement detection */
#define PREDICTION_INTERVAL_COEFFICIENT 1.96

/* orientation change detection */
#define NEW_MEASURE_MINIMAL_DEVIATION_COEFF 20.0

VarioCalibration Calibration;

#if defined(SDCARD_OUTPUT) && defined(HAVE_SDCARD)
//****************************************************************************************************************************
void VarioCalibration::writeNumber(int16_t number) {
//****************************************************************************************************************************
  valueDigit.begin((long)number);
  while( valueDigit.available() ) {
    file.write( valueDigit.get() );
  }
}
#endif //SDCARD_OUTPUT

/**************************/
void VarioCalibration::startMeasure(void) {

}

uint8_t VarioCalibration::readRawAccel(int16_t* accel, int32_t* quat) {


  return 0;
}


/*******************************/
void VarioCalibration::makeMeasureStep(void) {

}


/* return standard deviation */
double VarioCalibration::getAccelMeasure(int16_t* accelMeasure) {
  return 0;
}

//****************************************************************************************************************************
void VarioCalibration::Begin(void)
//****************************************************************************************************************************
{

}
