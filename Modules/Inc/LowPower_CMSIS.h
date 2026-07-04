#ifndef LOWPOWER_CMSIS_H
#define LOWPOWER_CMSIS_H

#include <stdint.h>
#include "stm32f4xx.h"

/* Güç Modlarına Giriş */
void LowPower_EnterSleep(void);                 // Sleep mode (WFI)
void LowPower_EnterStop(void);                  // Stop mode + uyanma sonrası clock restore
void LowPower_EnterStandby(void);               // Standby mode (uyanma = reset)

/* Yardımcı Fonksiyonlar */
uint8_t LowPower_WokeFromStandby(void);         // Standby'dan mı uyanıldı? (SBF flag)
void LowPower_ConfigWakeupPin(void);            // PA0 WKUP pin yapılandırması (Standby için)

#endif /* LOWPOWER_CMSIS_H */
