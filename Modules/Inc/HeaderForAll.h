#ifndef HEADERFORALL_H
#define HEADERFORALL_H

/*
  Modül Seçimi
  Aktif etmek istediğin modülü 1 yap, diğerlerini 0 bırak.
  Aynı anda birden fazla modül aktif edilebilir.
*/
#define ADC_CMSIS             0
#define DAC_CMSIS             0
#define PWM_CMSIS             0
#define GPIO_Interrupt_CMSIS  0
#define TIMER_CMSIS           0
#define UART_CMSIS            0
#define I2C_CMSIS             0
#define SPI_CMSIS             0
#define SYSTICK_CMSIS         0
#define DMA_CMSIS             0 // Bağımlılıklar: ADC_CMSIS, UART_CMSIS
#define WATCHDOG_CMSIS		  0
#define RTC_CMSIS             0
#define FLASH_CMSIS           0
#define LOW_POWER_CMSIS		  1


/*
  Clock — her zaman dahil
*/
#include <CLOCK_CMSIS.h>


/*
 Seçili modüller
*/
#if ADC_CMSIS
  #include <ADC_CMSIS.h>
#endif

#if DAC_CMSIS
  #include <DAC_CMSIS.h>
#endif

#if PWM_CMSIS
  #include <PWM_CMSIS.h>
#endif

#if GPIO_Interrupt_CMSIS
  #include <GPIO_Interrupt_CMSIS.h>
#endif

#if TIMER_CMSIS
  #include <Timer_CMSIS.h>
#endif

#if UART_CMSIS
  #include <UART_CMSIS.h>
#endif

#if I2C_CMSIS
  #include <I2C_CMSIS.h>
#endif

#if SPI_CMSIS
  #include <SPI_CMSIS.h>
#endif

#if SYSTICK_CMSIS
  #include <SYSTICK_CMSIS.h>
#endif

#if DMA_CMSIS
  #include <DMA_CMSIS.h>
#endif

#if WATCHDOG_CMSIS
  #include <Watchdog_CMSIS.h>
#endif

#if RTC_CMSIS
  #include <RTC_CMSIS.h>
#endif

#if FLASH_CMSIS
  #include <Flash_CMSIS.h>
#endif

#if LOW_POWER_CMSIS
#include <LowPower_CMSIS.h>
#endif

#endif /* HEADERFORALL_H */
