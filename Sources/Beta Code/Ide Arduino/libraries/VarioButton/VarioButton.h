#ifndef _VARIOBUTTON_H_
#define _VARIOBUTTON_H_

#if defined(ESP32)
#include <Arduino.h>
#include <DebugConfig.h>
#include "HardwareConfig.h"
#include "Button.h"

#include <VarioData.h>

#ifdef HAVE_SDCARD
#include <sdcardHAL.h>
#endif //HAVE_SDCARD

#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
#include <kalmanvert.h>
#include <AglManager.h>
#endif //HAVE_SDCARD && HAVE_GPS

#define DEBOUNCE_MS 30

class VARIOButton
{

public:
	void begin();
	void update();

	void setWakeupButton(uint8_t button);

	// Button API
	Button BtnA = Button(BUTTON_A_PIN, true, DEBOUNCE_MS);
	Button BtnB = Button(BUTTON_B_PIN, true, DEBOUNCE_MS);
	Button BtnC = Button(BUTTON_C_PIN, true, DEBOUNCE_MS);

	// SPEAKER

private:
	uint8_t _wakeupPin;
};

class VARIOButtonScheduleur
{
public:
	void update();
	void Set_StatePage(uint8_t state);
	uint8_t Get_StatePage(void);

private:
	uint8_t StatePage = STATE_PAGE_INIT;
	bool _stateBA = false;
	bool _stateBB = false;
	bool _stateBC = false;
	void treatmentBtnA(bool Debounce);
	void treatmentBtnB(bool Debounce);
	void treatmentBtnC(bool Debounce);

	void treatmentBtnB3S(bool Debounce);
	void treatmentBtnA2S(bool Debounce);

#ifdef HAVE_WIFI
	void WifiServeur(void);
	static void startWifi(void * pvParameters);
#endif //HAVE_WIFI

#ifdef HAVE_SDCARD
#ifdef SDFAT_LIB
	void printDirectory(SdFile dir, int numTabs);
#else //SDFAT_LIB
	void printDirectory(File dir, int numTabs);
#endif
#endif //HAVE_SDCARD
};

extern VARIOButton VarioButton;
extern VARIOButtonScheduleur ButtonScheduleur;

#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
extern Kalmanvert kalmanvert;
extern AglManager aglManager;
#endif

extern uint8_t variometerState;
extern void createSDCardTrackFile(void);

#else
#error “This library only supports boards with ESP32 processor.”
#endif

#endif
