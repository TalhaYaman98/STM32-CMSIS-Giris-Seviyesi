# STM32 CMSIS Examples

[![STM32F4](https://img.shields.io/badge/STM32-F4%20Discovery-orange.svg)](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)
[![CMSIS](https://img.shields.io/badge/CMSIS-Core%20%26%20Device-green.svg)](https://arm-software.github.io/CMSIS_5/Core/html/index.html)

Bu repository, STM32 mikrodenetleyiciler ile **HAL kütüphanesine bağımlılığı minimuma indirerek**, doğrudan **CMSIS (Cortex Microcontroller Software Interface Standard)** tabanlı programlama örneklerini içerir.

## 🎯 Amaç

STM32 programlamaya yeni başlayanların donanımın çalışma mantığını **en temel seviyede** öğrenmesini sağlamak:

- ✅ **Donanım kayıt seviyesinde** nasıl çalıştığını anlamak
- ✅ HAL gibi **yüksek seviyeli kütüphanelere bağımlılığı** azaltmak
- ✅ **Gerçek donanım kontrolü** mantığını kavramak
- ✅ **Register manipülasyonu** ile programlama yapmayı öğrenmek

## 📚 Mevcut Örnekler

| Konu | Dosyalar | Açıklama | Durum |
|------|----------|----------|-------|
| **Clock** | `Clock_CMSIS.c/h` | HSE + PLL ile 168 MHz sistem saati | ✅ Hazır |
| **GPIO** | `GPIO_Interrupt_CMSIS.c/h` | Dijital giriş/çıkış, LED kontrolü, buton okuma | ✅ Hazır |
| **EXTI** | `GPIO_Interrupt_CMSIS.c/h` | Harici kesme işlemleri, interrupt handling | ✅ Hazır |
| **ADC** | `ADC_CMSIS.c/h` | Analog-Dijital dönüşüm, PA0 kanal okuma | ✅ Hazır |
| **DAC** | `DAC_CMSIS.c/h` | Dijital-Analog dönüşüm, analog sinyal üretimi | ✅ Hazır |
| **PWM** | `PWM_CMSIS.c/h` | TIM4 ile darbe genişlik modülasyonu | ✅ Hazır |
| **Timer** | `Timer_CMSIS.c/h` | TIM2 ile 1 Hz güncelleme kesmesi | ✅ Hazır |
| **UART** | `UART_CMSIS.c/h` | USART2 seri haberleşme, TX/RX | ✅ Hazır |
| **I2C** | `I2C_CMSIS.c/h` | I2C1 master write/read (tek ve çok byte) | ✅ Hazır |
| **SPI** | `SPI_CMSIS.c/h` | SPI1 master full-duplex TX/RX (PA4–PA7) | ✅ Hazır |
| **SysTick** | `SysTick_CMSIS.c/h` | ms/us gecikme, timestamp (GetTick) | ✅ Hazır |
| **DMA** | `DMA_CMSIS.c/h` | DMA1 UART TX, DMA2 ADC circular transfer | ✅ Hazır |
| **Watchdog** | `Watchdog_CMSIS.c/h` | IWDG (bağımsız) ve WWDG (pencere) reset koruması | ✅ Hazır |
| **RTC** | `RTC_CMSIS.c/h` | Gerçek zamanlı saat, alarm ve zaman damgası (HSE/8) | ✅ Hazır |
| **Flash** | `Flash_CMSIS.c/h` | Dahili Flash okuma/yazma, sektör silme | ✅ Hazır |
| **Low Power** | `LowPower_CMSIS.c/h` | Sleep, Stop ve Standby güç tasarrufu modları | ✅ Hazır |
| **CAN** | — | Controller Area Network haberleşme protokolü | 🔜 Yakında |

## 🗂️ Proje Yapısı

```
STM32-CMSIS-Examples/
├── main.c                        # Ana program girişi
├── stm32f4xx_it.c                # Interrupt handler'ları (SysTick_Handler dahil)
├── HeaderForAll.h                # Modül seçim başlığı (merkezi include)
├── Clock_CMSIS.c / .h            # Sistem saati (HSE + PLL, 168 MHz)
├── ADC_CMSIS.c / .h              # ADC — PA0 analog okuma
├── DAC_CMSIS.c / .h              # DAC — PA4 analog çıkış
├── PWM_CMSIS.c / .h              # PWM — TIM4 CH1 (PD12)
├── Timer_CMSIS.c / .h            # Timer — TIM2 1 Hz IRQ (PD12 LED)
├── UART_CMSIS.c / .h             # UART — USART2 (PA2/PA3)
├── GPIO_Interrupt_CMSIS.c / .h   # GPIO + EXTI0 (PA0 buton, PD12 LED)
├── I2C_CMSIS.c / .h              # I2C1 master (PB6/PB7)
├── SPI_CMSIS.c / .h              # SPI1 master full-duplex (PA4/PA5/PA6/PA7)
├── SysTick_CMSIS.c / .h          # SysTick — ms/us gecikme, GetTick timestamp
├── DMA_CMSIS.c / .h              # DMA — UART TX (DMA1 S6 CH4), ADC (DMA2 S0 CH0)
├── Watchdog_CMSIS.c / .h         # IWDG (LSI) ve WWDG (APB1, pencere mantığı)
├── RTC_CMSIS.c / .h              # RTC — saat/tarih, alarm A, timestamp (HSE/8)
├── Flash_CMSIS.c / .h            # Flash — sektör silme, 32-bit okuma/yazma
└── LowPower_CMSIS.c / .h         # Low Power — Sleep, Stop, Standby modları
```

### HeaderForAll.h — Modül Seçimi

Tüm modüller `HeaderForAll.h` içinden merkezi olarak yönetilir. Kullanmak istediğiniz modülü `1` yapın, diğerlerini `0` bırakın:

```c
#define ADC_CMSIS             0
#define DAC_CMSIS             0
#define PWM_CMSIS             0
#define GPIO_Interrupt_CMSIS  0
#define TIMER_CMSIS           0
#define UART_CMSIS            0
#define I2C_CMSIS             0
#define SPI_CMSIS             0
#define SYSTICK_CMSIS         0
#define DMA_CMSIS             0
#define WATCHDOG_CMSIS        0
#define RTC_CMSIS             0
#define FLASH_CMSIS           0
#define LOWPOWER_CMSIS        0
```

`main.c` içindeki başlatma ve döngü kodları bu makrolara göre koşullu derleme (`#if`) ile aktif hale gelir. Clock modülü her zaman dahildir.

> ⚠️ DMA modülü UART ve ADC modüllerine bağımlıdır. `DMA_CMSIS` aktifken `ADC_CMSIS` ve `UART_CMSIS` de `1` yapılmalıdır.

> ⚠️ Low Power modülünde Sleep/Stop testleri için EXTI0 uyandırma kaynağı gerekir; `GPIO_Interrupt_CMSIS` de `1` yapılmalıdır.

## 📌 Pin Haritası

| Modül | Pin | Fonksiyon |
|-------|-----|-----------|
| ADC | PA0 | Analog giriş (Kanal 0) |
| DAC | PA4 | Analog çıkış (Kanal 1) |
| PWM | PD12 | TIM4 CH1 PWM çıkışı |
| Timer | PD12 | LED (1 Hz toggle) |
| UART TX | PA2 | USART2 TX (AF7) |
| UART RX | PA3 | USART2 RX (AF7) |
| I2C SCL | PB6 | I2C1 SCL (AF4) |
| I2C SDA | PB7 | I2C1 SDA (AF4) |
| EXTI | PA0 | Buton girişi (rising edge) |
| LED | PD12 | Çıkış (EXTI / Timer / SysTick / Watchdog testi) |
| SPI CS | PA4 | SPI1 Chip Select (Software) |
| SPI SCK | PA5 | SPI1 Saat (AF5) |
| SPI MISO | PA6 | SPI1 Master In Slave Out (AF5) |
| SPI MOSI | PA7 | SPI1 Master Out Slave In (AF5) |
| RTC Timestamp | PC13 | RTC_TAMP1 / RTC_TS — timestamp tetikleme pini |
| WKUP | PA0 | Standby'dan uyandırma pini (user button) |

## ⏱️ Saat Konfigürasyonu

Tüm örnekler aşağıdaki saat hiyerarşisini kullanır:

| Bus | Frekans | Notlar |
|-----|---------|--------|
| SYSCLK | 168 MHz | HSE (8 MHz) + PLL |
| AHB (HCLK) | 168 MHz | GPIO, DMA, SysTick, Flash |
| APB2 (PCLK2) | 84 MHz | USART1, ADC, SPI1, TIM1 |
| APB1 (PCLK1) | 42 MHz | I2C, USART2, TIM2/4, WWDG |
| Timer clock (APB1 × 2) | 84 MHz | TIM2, TIM4 |
| SysTick clock | 168 MHz | AHB kaynağı seçili (CLKSOURCE = 1) |
| LSI (IWDG) | ~32 kHz | Sistem clock'undan bağımsız dahili osilatör |
| RTC clock | 1 MHz | HSE / RTCPRE(8) → PREDIV_A=124, PREDIV_S=7999 → 1 Hz |

## 🐕 Watchdog Modülü — Detaylar

### IWDG (Independent Watchdog)
- Clock kaynağı **LSI (~32 kHz)**, sistem clock'undan tamamen bağımsızdır.
- Bir kez başlatıldıktan sonra **durdurulamaz**, yalnızca reset ile kapanır.
- `IWDG_Init(prescaler, reload)` ile timeout ayarlanır: `T = (reload+1) / (LSI / prescaler_bölücü)`
- `IWDG_Refresh()` periyodik olarak çağrılmalı (timeout'tan kısa aralıklarla).

### WWDG (Window Watchdog)
- Clock kaynağı **APB1**, sistem clock'una bağlıdır.
- "Pencere" mantığı: refresh hem **çok erken** (sayaç > W) hem **çok geç** (sayaç < 0x40) yapılırsa reset tetiklenir.
- `WWDG_Init(t_value, w_value, prescaler)` ile pencere ve timeout ayarlanır.
- `WWDG_Refresh(t_value)` **sadece** `(WWDG->CR & WWDG_CR_T) <= w_value` koşulu sağlandığında çağrılmalıdır.
- ⚠️ `WDGA` biti set edildikten sonra yazılımla temizlenemez (donanımsal kilit, sadece reset ile kalkar).

### Reset Sebebini Doğrulama
```c
uint32_t reset_reason = RCC->CSR;
if(reset_reason & RCC_CSR_IWDGRSTF) { /* IWDG kaynaklı reset */ }
if(reset_reason & RCC_CSR_WWDGRSTF) { /* WWDG kaynaklı reset */ }
RCC->CSR |= RCC_CSR_RMVF;   // Flag'leri temizle
```

### Debug Notu
```c
DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP | DBGMCU_APB1_FZ_DBG_WWDG_STOP;
```
Bu satır olmadan, debugger CPU'yu durdurduğunda watchdog sayaçları çalışmaya devam eder ve beklenmedik resetlere yol açar.

## 🕐 RTC Modülü — Detaylar

### Clock Kaynağı
- **HSE / RTCPRE(8) = 1 MHz** → PREDIV_A=124, PREDIV_S=7999 → **1 Hz ck_spre**
- Backup domain'de çalışır; VDD kesilse bile VBAT pinine pil bağlanırsa saat korunur.

### Fonksiyonlar
- `RTC_Init()` — Clock kaynağı, prescaler, 24h format, timestamp yapılandırması. RTC zaten çalışıyorsa (RTCEN=1) init atlanır, saat korunur.
- `RTC_SetTime(hour, min, sec)` / `RTC_SetDate(year, month, day, weekday)` — BCD dönüşümü ile TR/DR register'larına yazar.
- `RTC_GetTime()` / `RTC_GetDate()` — Shadow register'lardan okur. GetTime() ardından GetDate() çağrılmalıdır (tutarlılık).
- `RTC_SetAlarm(hour, min, sec)` — Alarm A, EXTI Line 17 + NVIC ile interrupt desteği.
- `RTC_GetTimestamp(hour, min, sec)` — TSF flag kontrolü ile TSTR'dan timestamp okur.

### BCD Formatı
RTC register'ları BCD (Binary Coded Decimal) formatında çalışır. Fonksiyonlar decimal↔BCD dönüşümünü otomatik yapar.

## 💾 Flash Modülü — Detaylar

### Sektör Haritası (STM32F407, 1 MB)

| Sektör | Adres Başlangıcı | Boyut | Önerilen Kullanım |
|--------|-----------------|-------|-------------------|
| 0 | `0x08000000` | 16 KB | Uygulama kodu (interrupt vector) |
| 1–3 | `0x08004000` | 16 KB | Uygulama kodu |
| 4 | `0x08010000` | 64 KB | Uygulama kodu |
| 5 | `0x08020000` | 128 KB | Uygulama kodu |
| 6–11 | `0x08040000+` | 128 KB | **Kullanıcı verisi** |

### Fonksiyonlar
- `Flash_Unlock()` / `Flash_Lock()` — 0x45670123 + 0xCDEF89AB anahtar çifti ile kilit açma/kapama.
- `Flash_EraseSector(sector)` — Belirtilen sektörü siler (tüm bitler 1 olur). Yazmadan önce zorunludur.
- `Flash_WriteWord(address, data)` — 32-bit (word) yazma, PG bit kontrolü ile.
- `Flash_WriteBuffer(address, data, length)` — Ardışık 32-bit veri dizisi yazma.
- `Flash_ReadWord(address)` — 32-bit okuma (unlock gerektirmez).
- `Flash_ReadBuffer(address, data, length)` — Ardışık 32-bit veri dizisi okuma.

### Kritik Uyarılar
> ⚠️ Sektör **0–5 silinmemelidir** — uygulama kodu bu sektörlerde bulunur, silinirse sistem çöker.

> ⚠️ Flash'a yazmadan önce o adres **mutlaka silinmiş** olmalıdır. 0→1 yazımı mümkün değildir.

> ⚠️ Aynı adrese **iki kez yazma** ECC hatasına yol açar. Her yazma öncesi sektör silinmelidir.

> ⚠️ PSIZE = 32-bit (VDD = 3.3V) seçilmiştir. Farklı voltajda çalışıyorsanız PSIZE değerini güncelleyin.

## 🔋 Low Power Modülü — Detaylar

### Güç Modları Karşılaştırması

| Mod | CPU | Clock'lar | SRAM | Uyandırma | Tipik Akım |
|-----|-----|-----------|------|-----------|------------|
| **Sleep** | Durur | Çevre birimleri çalışır | Korunur | Herhangi bir interrupt | ~mA |
| **Stop** | Durur | Hepsi durur (LSI/LSE hariç) | Korunur | Yalnızca EXTI line | ~µA |
| **Standby** | Kapalı | Hepsi kapalı (backup domain hariç) | **Silinir** | WKUP pini, RTC alarm, IWDG | ~2 µA |

### Fonksiyonlar
- `LowPower_EnterSleep()` — SLEEPDEEP=0 + `__WFI()`. En hızlı uyanma; herhangi bir interrupt uyandırır.
- `LowPower_EnterStop()` — PDDS=0, LPDS=1 + `__WFI()`. Uyanma sonrası otomatik `Clock_Init()` çağrılır (HSI→168 MHz restore).
- `LowPower_EnterStandby()` — PDDS=1, WUF temizleme + `__WFI()`. Uyanma reset gibi davranır, `main()` baştan başlar.
- `LowPower_WokeFromStandby()` — `PWR_CSR_SBF` bayrağı ile standby uyanması vs normal reset ayırt edilir.
- `LowPower_ConfigWakeupPin()` — PA0'ı WKUP pini olarak etkinleştirir (EWUP biti).

### Kritik Uyarılar
> ⚠️ **Stop modunda** TIM/UART gibi normal interrupt'lar uyandıramaz (clock'ları durmuştur); yalnızca **EXTI line** kaynakları (buton, RTC alarm) uyandırır. Uyandırma kaynağı yapılandırılmadan Stop'a girilirse yalnızca reset ile çıkılabilir.

> ⚠️ **Stop'tan uyanınca** sistem HSI (16 MHz) ile çalışır; HSE + PLL yeniden başlatılmalıdır. `LowPower_EnterStop()` bunu otomatik yapar.

> ⚠️ **Standby'dan uyanma** reset gibi davranır: SRAM ve tüm değişkenler silinir. Kalıcı veri backup register (RTC->BKPxR) veya Flash'ta saklanmalıdır.

> ⚠️ **Standby'a girmeden önce** WUF bayrağı temizlenmelidir (CWUF); aksi halde MCU anında geri uyanır.

> ⚠️ **HAL SysTick uyarısı:** HAL_Init() SysTick'i 1 ms interrupt'a ayarlar; Sleep modunda `__WFI()` en fazla 1 ms uyur. Uzun uyku için SysTick geçici durdurulmalıdır.

> ⚠️ `EWUP` set edildikten sonra PA0 normal GPIO olarak kullanılamaz; pin WKUP fonksiyonuna tahsis edilir.

## ⚠️ Önemli Notlar

- Bu örnekler **eğitim amaçlıdır** ve ticari projelerde kullanılmadan önce test edilmelidir
- **Register seviyesinde** programlama yaptığımız için dikkatli olmak gerekir
- `main.c` HAL ile oluşturulmuş bir proje iskeletini temel alır; çevresel birim başlatmaları CMSIS ile yapılmaktadır
- Her modül **bağımsız** çalışacak şekilde tasarlanmıştır
- **STM32F4 Discovery** kartı için optimize edilmiştir
- SPI modülünde PA4 hem DAC çıkışı hem CS pini olarak atanmıştır; DAC ve SPI aynı anda kullanılmamalıdır
- SysTick modülü HAL ile birlikte çalışır; `stm32f4xx_it.c` içindeki `SysTick_Handler`'a `tick_count++` eklenmelidir
- DMA modülü UART ve ADC modüllerine bağımlıdır; birlikte aktif edilmelidir
- WWDG_Refresh() çağrılırken sayaç ve WDGA biti **tek yazma işleminde** birlikte yazılmalıdır; iki adımlı yazım T6 bitinin anlık 0 olmasına ve anında resete yol açar
- RTC modülü backup domain'de çalışır; `PWR_CR_DBP` biti set edilmeden RTC register'larına yazılamaz
- Low Power Stop/Standby modlarında debugger bağlantısı kopabilir; `DBGMCU->CR` içindeki `DBG_STOP`/`DBG_STANDBY` bitleri debug sırasında bağlantıyı korur

## 📚 Faydalı Kaynaklar

- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [STM32F407 Datasheet](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)
- [CMSIS Documentation](https://arm-software.github.io/CMSIS_5/Core/html/index.html)
- [STM32 Programming Manual](https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf)

---

⭐ Bu projeyi beğendiyseniz **star** vermeyi unutmayın!

🐛 Hata bulursanız veya öneriniz varsa **issue** açmaktan çekinmeyin.

💡 Yeni örnek önerilerinizi **discussions** bölümünde paylaşabilirsiniz.
