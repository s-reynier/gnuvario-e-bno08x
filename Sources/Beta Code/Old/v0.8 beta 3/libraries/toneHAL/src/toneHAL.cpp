/* toneHAL -- tone HAL
 *
 * Copyright 2019 Jean-philippe GOI
 * 
 * This file is part of toneHAL.
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
/*                           Libraries ToneHal                                   */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0    20/01/19                                                            */
/*    1.1    24/01/19   Réecriture des classes                                   */
/*                      répartition en plusieurs fichiers                        */
/*    1.2    26/01/19   Modifications mineures                                   */
/*    1.3    09/02/19   Ajout TONEHAL_EXTENDED_VOLUME							               */
/*    1.4    02/03/19   Ajout ESP32                                              */
/*    1.4.1  12/03/19   Modifications mineures								                   */
/*    1.4.2  16/03/19   Ajout description et licence en début de fichier         */
/*    1.4.3  08/04/19   ToneDAC pour l'ESP32                                     */
/*    1.4.4  25/04/19 	Modification DebugConfig.h et HardwareConfig.h           */
/*    1.4.5  05/06/19 	Déclaration de l'instance dans toneHAL.cpp						   */
/*    1.4.6  10/06/19 	Ajout gestion ampli class D                              */ 
/*    1.4.7  02/07/19   Ajout isMute()                                           */
/*    1.5.0  30/08/19		Ajout gestion commande ampli													   */
/*    1.5.1  03/11/19		Modification des fonctions enableAmpli 									 */
/*    1.5.2  10/01/20   Ajout TONEDACTIMER																			 */
/*    1.5.3  14/06/20   Ajout update et TONEXTDAC                                */
/*                                                                               */
/*********************************************************************************/

/***********************************************************************************/
/*  ProMini :                                                                      */
/*                PWM 1 pin                         not yet developed              */
/*                PWM AC 2 pins                     OK                             */
/*                DAC                               Not available                  */
/*                I2S                               Not available                  */
/*                                                                                 */
/*  MKZERO                                                                         */
/*                PWM 1 pin                         OK                             */
/*                PWM 2 pins                        OK                             */
/*                DAC                               OK                             */
/*                I2S                               not yet developed              */
/*                                                                                 */
/*  ESP32         PWM 1 pin                         OK                             */
/*                PWM 2 pins                        not yet developed              */
/*                DAC                               in development                 */
/*                I2S                               not yet developed              */
/*                                                                                 */
/***********************************************************************************/                

#include "toneHAL.h"
#include <Arduino.h>
#include <HardwareConfig.h>
#include <DebugConfig.h>

#if defined (TONEHAL_EXTENDED_VOLUME)

/***********************************/
void ToneHal::setVolume(uint8_t newVolume) {
/***********************************/

  _volume = newVolume;
}

/***********************************/
uint8_t ToneHal::getVolume() {
/***********************************/
/*  if (_toneMuted) return 0;
	else            return _volume;*/
  return _volume;	
}

/***********************************/
void ToneHal::mute(bool newMuteState) {
/***********************************/
  /* stop tone if needed */
  if( newMuteState ) {
#if defined(HAVE_AUDIO_AMPLI) && defined(AUDIO_AMPLI_LOWPOWER)	
		if (PIN_AUDIO_AMP_ENA != -1) AUDIO_AMP_DISABLE();
#endif //HAVE_AUDIO_AMPLI
		
    noTone();
  }

  /* save */
  _toneMuted = newMuteState;
}

bool ToneHal::isMute(void) { 
  return _toneMuted;
}

#endif //TONEHAL_EXTENDED_VOLUME

#ifdef HAVE_AUDIO_AMPLI

/***********************************/
void ToneHal::AUDIO_AMP_DISABLE(void) {
/***********************************/
#ifdef SOUND_DEBUG
    SerialPort.println("AUDIO_AMP_DESABLE");
#endif //BUTTON_DEBUG

#ifdef AUDIO_AMP_MODE_LOW
	digitalWrite(PIN_AUDIO_AMP_ENA,HIGH);
#else
	digitalWrite(PIN_AUDIO_AMP_ENA,LOW);
#endif
}

/***********************************/
void ToneHal::AUDIO_AMP_ENABLE(void) {
/***********************************/
	pinMode(PIN_AUDIO_AMP_ENA,OUTPUT);
#ifdef SOUND_DEBUG
    SerialPort.print("AUDIO_AMP_ENABLE / pin : ");
		SerialPort.println(PIN_AUDIO_AMP_ENA);
#endif //BUTTON_DEBUG

#ifdef AUDIO_AMP_MODE_LOW

#ifdef SOUND_DEBUG
    SerialPort.println("AUDIO_AMP_ENABLE LOW");
#endif //BUTTON_DEBUG

	digitalWrite(PIN_AUDIO_AMP_ENA,LOW);
#else
#ifdef SOUND_DEBUG
    SerialPort.println("AUDIO_AMP_ENABLE HIGH");
#endif //BUTTON_DEBUG

	digitalWrite(PIN_AUDIO_AMP_ENA,HIGH);

#endif
}

/***********************************/
void ToneHal::AUDIO_AMP_INIT(void) {
/***********************************/

//init Pin commande ampli
#ifdef SOUND_DEBUG
    SerialPort.println("AUDIO_AMP_INIT");
#endif //BUTTON_DEBUG

	if (PIN_AUDIO_AMP_ENA != -1) {
		pinMode(PIN_AUDIO_AMP_ENA,OUTPUT);
		
#ifdef SOUND_DEBUG
    SerialPort.print("pin : ");
		SerialPort.println(PIN_AUDIO_AMP_ENA);
#endif //BUTTON_DEBUG
		
#ifdef AUDIO_AMP_MODE_LOW
	  digitalWrite(PIN_AUDIO_AMP_ENA,HIGH);
#else
	  digitalWrite(PIN_AUDIO_AMP_ENA,LOW);
#endif	
  }
}

#endif //HAVE_AUDIO_AMPLI

/***********************************/
void ToneHal::enableAmpli(void) {
/***********************************/
#ifdef HAVE_AUDIO_AMPLI
	AUDIO_AMP_ENABLE();
#endif //HAVE_AUDIO_AMPLI
}

/***********************************/
void ToneHal::disableAmpli(void) {
/***********************************/
#ifdef HAVE_AUDIO_AMPLI
	AUDIO_AMP_DISABLE();
#endif //HAVE_AUDIO_AMPLI
}


ToneHAL toneHAL;
