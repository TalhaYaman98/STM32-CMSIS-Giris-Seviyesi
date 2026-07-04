/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "HeaderForAll.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if WATCHDOG_CMSIS
#define WDG_TEST_SELECT   4   /* <-- Test numarasını buradan değiştir (1-4) */
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if WATCHDOG_CMSIS
volatile uint8_t wwdg_current = 0;
volatile uint32_t reset_reason = 0;
volatile uint8_t  test_a_locked = 0;   /* TEST A için: kilitlenme durumu göstergesi */
#endif

#if RTC_CMSIS
uint8_t hour, min, sec;
uint8_t year, month, day, weekday;

uint8_t ts_h, ts_m, ts_s;
#endif

#if FLASH_CMSIS
uint32_t read_data[4]  = {0};
uint32_t write_data[4] = {0xDEADBEEF, 0x12345678, 0xAABBCCDD, 0x11223344};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

#if ADC_CMSIS
  GPIO_PA0_Analog_Init();
  ADC1_Init();
#endif

#if DAC_CMSIS
  GPIO_PA4_Analog_Init();
  DAC_Init();
#endif

#if PWM_CMSIS
  GPIO_PD12_PWM_Init();
  TIM4_PWM_Init(1000, 50);
#endif

#if GPIO_Interrupt_CMSIS
  GPIO_Interrupt_Init();
  Exti0_Init();
#endif

#if TIMER_CMSIS
  GPIO_PD12_Timer_Init();
  TIM2_1HZ_Init();
#endif

#if UART_CMSIS
  GPIO_UART_Init();
  USART2_Init();
  USART2_Send_String("UART OK\r\n");
#endif

#if I2C_CMSIS
  I2C1_Init();
#endif

#if SPI_CMSIS
  GPIO_SPI1_Init();
  SPI1_Init();
#endif

#if SYSTICK_CMSIS
  SysTick_Init();

  // PD12 GPIO Init (SysTick örneğini görmek için)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;          // GPIOD clock enable
  (void)RCC->AHB1ENR;                            // Bus senkronizasyonu
  GPIOD->MODER  &= ~(3 << (12 * 2));             // PD12 mod bitlerini temizle
  GPIOD->MODER  |=  (1 << (12 * 2));             // PD12 → Output (01)
  GPIOD->OTYPER &= ~(1 << 12);                   // Push-pull
  GPIOD->OSPEEDR|=  (3 << (12 * 2));             // Very high speed
  GPIOD->PUPDR  &= ~(3 << (12 * 2));             // No pull
  GPIOD->BSRR    =  (1 << (12 + 16));            // Başlangıçta LED kapalı
#endif

#if DMA_CMSIS
  // UART DMA örneği
//  GPIO_UART_Init();
//  USART2_Init();
  DMA_UART_Init();

  uint8_t msg[] = "DMA ile UART gonderimi!\r\n";
  DMA_UART_Transmit(msg, sizeof(msg) - 1);

  // ADC DMA örneği
//  GPIO_PA0_Analog_Init();
//  ADC1_Init();
  DMA_ADC_Init();

  uint16_t adc_buf[10] = {0};
  DMA_ADC_Start(adc_buf, 10);
#endif

#if WATCHDOG_CMSIS
  /* Debug sırasında watchdog'ların durmasını sağla */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP | DBGMCU_APB1_FZ_DBG_WWDG_STOP;

  /* Reset sebebini oku ve Watch'tan izle: reset_reason */
  reset_reason = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;   /* Flag'leri temizle (sonraki teste hazırlık) */

  #if (WDG_TEST_SELECT == 1)
    /* TEST A: IWDG init — IWDG_clk = 32000/64 = 500 Hz, RLR=2499 -> ~5 sn timeout */
    IWDG_Init(4, 2499);

  #elif (WDG_TEST_SELECT == 2)
    /* TEST B1: WWDG init — refresh while döngüsünde hemen pencere ihlali yapacak */
    WWDG_Init(0x7F, 0x5F, 3);

  #elif (WDG_TEST_SELECT == 3)
    /* TEST B2: WWDG init — refresh hiç çağrılmayacak, timeout reseti beklenecek */
    WWDG_Init(0x7F, 0x5F, 3);

  #elif (WDG_TEST_SELECT == 4)
    /* TEST B3: WWDG init — pencere içinde doğru refresh yapılacak */
    WWDG_Init(0x7F, 0x5F, 3);

    // PD12 GPIO Init
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;          // GPIOD clock enable
    (void)RCC->AHB1ENR;                            // Bus senkronizasyonu
    GPIOD->MODER  &= ~(3 << (12 * 2));             // PD12 mod bitlerini temizle
    GPIOD->MODER  |=  (1 << (12 * 2));             // PD12 → Output (01)
    GPIOD->OTYPER &= ~(1 << 12);                   // Push-pull
    GPIOD->OSPEEDR|=  (3 << (12 * 2));             // Very high speed
    GPIOD->PUPDR  &= ~(3 << (12 * 2));             // No pull
    GPIOD->BSRR    =  (1 << (12 + 16));            // Başlangıçta LED kapalı

  #endif
#endif

#if RTC_CMSIS
    RTC_Init();

    // Saat ve tarih ayarla (bir kez yapılır, sonra RTC bağımsız çalışır)
    RTC_SetTime(14, 30, 0);           // 14:30:00
    RTC_SetDate(25, 6, 28, 6);        // 2025, Haziran, 28, Cumartesi (6)

    // Alarm: 14:30:10'da tetiklenecek
    RTC_SetAlarm(14, 30, 10);
#endif

#if FLASH_CMSIS
    // Kullanıcı verisi için Sektör 6 seçildi (0x08040000, 128 KB)
    // UYARI: Uygulama kodunun bulunduğu sektörü silmeyin!

    // 1. Sektörü sil
    Flash_EraseSector(6);

    // 2. Veri yaz
    Flash_WriteBuffer(FLASH_SECTOR6_ADDR, write_data, 4);

    // 3. Veriyi geri oku ve doğrula
    Flash_ReadBuffer(FLASH_SECTOR6_ADDR, read_data, 4);
#endif

#if LOW_POWER_CMSIS
    // ===== Standby'dan uyanma kontrolü (main başında) =====
     if(LowPower_WokeFromStandby()){
         // Standby'dan uyanıldı — LED yak, log tut vb.
     }

     // PD12 LED init (görsel doğrulama için)
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
     (void)RCC->AHB1ENR;
     GPIOD->MODER  &= ~(3 << (12 * 2));
     GPIOD->MODER  |=  (1 << (12 * 2));
     GPIOD->BSRR    =  (1 << 12);              // LED aç — sistem çalışıyor göstergesi

     // ===== Örnek 1: Sleep Mode =====
     // EXTI0 (PA0 buton) yapılandırılmış olmalı (GPIO_Interrupt_CMSIS)
     // GPIO_Interrupt_Init();
     // Exti0_Init();
     // LowPower_EnterSleep();                 // Butona basınca uyanır

     // ===== Örnek 2: Stop Mode =====
     // GPIO_Interrupt_Init();
     // Exti0_Init();
     // GPIOD->BSRR = (1 << (12 + 16));        // LED kapat — uyumaya giriyor
     // LowPower_EnterStop();                  // Butona basınca uyanır
     // GPIOD->BSRR = (1 << 12);               // LED aç — uyandı (clock restore edildi)

     // ===== Örnek 3: Standby Mode =====
     // LowPower_ConfigWakeupPin();            // PA0 WKUP aktif
     // HAL_Delay(3000);                       // 3 sn bekle (gözlem için)
     // LowPower_EnterStandby();               // Butona basınca RESET gibi uyanır
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if ADC_CMSIS
    adc_value = ADC1_Read();
#endif

#if DAC_CMSIS
    DAC_Set_Voltage(1.65f);
    for (volatile int i = 0; i < 1000000; i++);
#endif

#if PWM_CMSIS
    /* PWM timer tabanlı çalışır, burada ek işlem gerekmez */
#endif

#if GPIO_Interrupt_CMSIS
    /* Interrupt tabanlı, burada ek işlem gerekmez */
#endif

#if TIMER_CMSIS
    __WFI();   /* Interrupt gelene kadar CPU'yu uyut */
#endif

#if UART_CMSIS
    char c = USART2_Read_Char();
    USART2_Send_Char(c);
#endif

#if I2C_CMSIS
    /* I2C işlemleri burada */
#endif

#if SPI_CMSIS
  uint8_t tx_data[] = {0x9F};          // Örnek komut (JEDEC ID okuma)
  uint8_t rx_data[3] = {0};

  SPI1_CS_Low();                        // CS aktif et
  SPI1_Transmit(tx_data, 1);           // Komut gönder
  SPI1_Receive(rx_data, 3);            // 3 byte oku
  SPI1_CS_High();                       // CS pasif et
#endif

#if SYSTICK_CMSIS
  // Örnek 1: delay_ms kullanımı
  GPIOD->BSRR = (1 << 12);          // PD12 LED aç
  delay_ms(500);                     // 500 ms bekle
  GPIOD->BSRR = (1 << (12 + 16));   // PD12 LED kapat
  delay_ms(500);                     // 500 ms bekle

  // Örnek 2: delay_us kullanımı
  GPIOD->BSRR = (1 << 12);          // PD12 LED aç
  delay_us(100);                     // 100 µs bekle
  GPIOD->BSRR = (1 << (12 + 16));   // PD12 LED kapat
  delay_us(100);                     // 100 µs bekle

  // Örnek 3: GetTick ile non-blocking zaman ölçümü
  uint32_t start = GetTick();
  // ... başka işler ...
  uint32_t elapsed = GetTick() - start;  // Geçen süre (ms)
#endif

#if DMA_CMSIS
  // adc_buf[] sürekli güncelleniyor, CPU serbest
  // UART transferi tamamlandıysa yeni mesaj gönderilebilir:
  // if(DMA1->HISR & DMA_HISR_TCIF6) { DMA_UART_Transmit(...); }
#endif

#if WATCHDOG_CMSIS

  #if (WDG_TEST_SELECT == 1)
    /* TEST A: 3. saniyede kasıtlı kilitlenme */
    static uint32_t test_a_counter = 0;

    if(!test_a_locked){
        IWDG_Refresh();
        HAL_Delay(1000);
        test_a_counter++;

        if(test_a_counter == 3){
            test_a_locked = 1;   /* Watch'tan bu değişimi gözlemleyebilirsin */
            while(1){
                __NOP();          /* IWDG_Refresh çağrılmıyor -> ~2 sn sonra reset */
            }
        }
    }

  #elif (WDG_TEST_SELECT == 2)
    /* TEST B1: Sayaç hâlâ W(0x5F)'nin üzerinde (0x7F), refresh -> ANINDA RESET */
    WWDG_Refresh(0x7F);
    while(1){ __NOP(); }   /* Reset olmazsa buraya takılı kalır (hata durumu) */

  #elif (WDG_TEST_SELECT == 3)
    /* TEST B2: Refresh çağrılmıyor, ~50 ms içinde timeout reseti bekleniyor */
    __NOP();

  #elif (WDG_TEST_SELECT == 4)
    wwdg_current = WWDG->CR & WWDG_CR_T;

    if(wwdg_current <= 0x5F){
        WWDG_Refresh(0x7F);
        GPIOD->ODR ^= (1 << 12);
    }
  #endif

#endif

#if RTC_CMSIS
    // Saat ve tarih oku
    RTC_GetTime(&hour, &min, &sec);
    RTC_GetDate(&year, &month, &day, &weekday);

    // Timestamp kontrolü
    RTC_GetTimestamp(&ts_h, &ts_m, &ts_s);

    HAL_Delay(1000);
#endif

#if FLASH_CMSIS
    // Flash işlemleri tamamlandı, ana döngüde ek işlem gerekmez
#endif

#if LOW_POWER_CMSIS
    // Low power işlemleri tamamlandı, ana döngüde ek işlem gerekmez
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
