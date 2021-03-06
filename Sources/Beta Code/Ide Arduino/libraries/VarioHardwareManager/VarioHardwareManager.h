/* VarioHardwareManager -- 
 *
 * Copyright 2020 MichelPa / Jpg63
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

/* 
 *********************************************************************************
 *                                                                               *
 *                           VarioHardwareManager                                *
 *                                                                               *
 *  version    Date     Description                                              *
 *    1.0    22/03/20                                                            *
 *    1.0.1  09/06/20   Ajout GnuSettings.BLUETOOTH_SEND_CALIBRATED_ALTITUDE     *
 *    1.0.2  25/12/20   Modif getCap                                             *
 *    1.0.3  11/04/21   Mofig getAlti                                            *
 *                                                                               *
 *********************************************************************************
 */

#ifndef VARIO_HARDWARE_MANAGER_H
#define VARIO_HARDWARE_MANAGER_H

#include <HardwareConfig.h>
#include <VarioPower.h>
#include <VarioSpeaker.h>
#include <VarioImu.h>
#include <VarioGps.h>

#ifdef HAVE_BLUETOOTH
#include <VarioBluetooth.h>
#endif


/*******************/
/* General objects */
/*******************/
#define VARIOMETER_STATE_INITIAL 0
#define VARIOMETER_STATE_DATE_RECORDED 1
#define VARIOMETER_STATE_CALIBRATED 2
#define VARIOMETER_STATE_FLIGHT_STARTED 3

class VarioHardwareManager
{
private:
	
	
#ifdef HAVE_BLUETOOTH
	VarioBluetooth varioBT;
#endif


public:
	VarioGps varioGps;
	VarioImu varioImu;
	VarioPower varioPower;
	VarioSpeaker varioSpeaker;

	VarioHardwareManager();
	void init();
	void initAlim();
	void initSpeaker();
	void initSound();
	void initImu();
	void initGps();
	bool initBt();
	void initButton();

	double firstAlti(void);
	bool updateData(void);

	double getAlti();
	double getTemp();
	double getAccel();
	int getBearing();
	void testInactivity(double velocity);

	bool updateBluetooth(double velocity, double alti, double altiCalibrated);
	bool updateGps(Kalmanvert kalmanvert);

	unsigned long time_deep_sleep, sleepTimeoutSecs;
};

//**************************************************
//Internal TEMPERATURE Sensor
//*************************************************

/* 
 *  https://circuits4you.com
 *  ESP32 Internal Temperature Sensor Example
 */

#ifdef __cplusplus
extern "C"
{
#endif

	uint8_t temprature_sens_read();

#ifdef __cplusplus
}
#endif

uint8_t temprature_sens_read();

extern VarioHardwareManager varioHardwareManager;

#endif //VARIO_HARDWARE_MANAGER_H