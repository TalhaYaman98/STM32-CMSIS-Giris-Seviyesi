#include "LowPower_CMSIS.h"

/* LowPower_EnterSleep — Sleep Mode
   CPU durur, çevre birimleri ve clock'lar çalışmaya devam eder.
   Uyandırma: Herhangi bir interrupt. */
void LowPower_EnterSleep(void){
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;                        // SLEEPDEEP = 0 → Sleep mode (Stop/Standby değil)
    __WFI();                                                     // Wait For Interrupt — CPU'yu uyut
}

/*

Amaç: CPU'yu Sleep moduna almak; interrupt gelene kadar uyur.
Referans: PM0214 — Bölüm 2.5 (Power management), RM0090 — Bölüm 5 (PWR).

Sleep Mode Genel Bilgi:
    En hafif güç tasarrufu modudur. Yalnızca CPU core clock'u durur;
    tüm çevre birimleri (TIM, UART, ADC, DMA...) çalışmaya devam eder.
    SRAM ve tüm register'lar korunur.
    Uyanma süresi en kısadır (birkaç cycle) — gerçek zamanlı sistemler
    için idealdir.

SCB->SCR (System Control Register, Cortex-M4 çekirdeği):
    SLEEPDEEP (bit 2): 0 = Sleep, 1 = Deep Sleep (Stop/Standby).
    Bu bit Cortex-M çekirdeğine hangi güç modunun hedeflendiğini bildirir.
    STM32 tarafında Stop mu Standby mi olacağı PWR->CR ile belirlenir;
    SLEEPDEEP=0 iken PWR ayarları önemsizdir, her zaman Sleep'e girilir.

__WFI() (Wait For Interrupt):
    Ne yapar: CPU'yu uyutan ARM instruction'ı (assembly: wfi).
    Herhangi bir interrupt (SysTick dahil!) CPU'yu uyandırır.
    Alternatif: __WFE() (Wait For Event) — event tabanlı uyandırma;
    interrupt handler çalıştırmadan uyanmak için kullanılır.

DİKKAT — SysTick uyarısı:
    HAL_Init() SysTick'i 1 ms interrupt üretecek şekilde yapılandırır.
    Bu durumda __WFI() en fazla 1 ms uyur (SysTick interrupt'ı uyandırır).
    Daha uzun uyku için SysTick geçici olarak durdurulabilir:
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // uyumadan önce
        __WFI();
        SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;  // uyandıktan sonra

*/

/* LowPower_EnterStop — Stop Mode
   Tüm clock'lar durur (LSI/LSE hariç). SRAM ve register'lar korunur.
   Uyandırma: EXTI line (buton, RTC alarm vb.)
   Uyanma sonrası: Sistem HSI ile çalışır → clock yeniden yapılandırılmalı! */
void LowPower_EnterStop(void){
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;                         // PWR clock enable
    (void)RCC->APB1ENR;

    PWR->CR &= ~PWR_CR_PDDS;                                    // PDDS = 0 → Stop mode (Standby değil)
    PWR->CR |= PWR_CR_LPDS;                                     // LPDS = 1 → Voltaj regülatörü low-power modda

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;                         // SLEEPDEEP = 1 → Deep sleep (Stop)

    __WFI();                                                     // Uyu — EXTI interrupt'ı uyandırır

    /* ===== Bu noktadan sonrası UYANDIKTAN SONRA çalışır ===== */

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;                        // SLEEPDEEP'i temizle (sonraki WFI'lar Sleep olsun)

    /* Stop'tan uyanınca sistem HSI (16 MHz) ile çalışır.
       HSE + PLL yeniden başlatılıp 168 MHz'e dönülmelidir. */
    Clock_Init();                                               // HSE + PLL → 168 MHz restore
}

/*

Amaç: MCU'yu Stop moduna almak; EXTI kaynaklı interrupt ile uyandırmak.

Stop Mode Genel Bilgi:
    1.2V domain'deki tüm clock'lar durur: HSI, HSE, PLL kapanır.
    SRAM ve tüm register içerikleri KORUNUR — uyanınca kaldığı yerden devam eder.
    Tipik akım tüketimi: ~µA mertebesi (LPDS=1 ile daha da düşük).
    Uyandırma kaynakları: Herhangi bir EXTI line'ı (GPIO butonu, RTC alarm,
    RTC wakeup timer, USB wakeup vb.) — normal interrupt'lar (TIM, UART) UYANDIRAMAZ
    çünkü clock'ları durmuştur.

PWR->CR bitleri:
    PDDS (bit 1): Power Down Deep Sleep.
        0 = Deep sleep'te Stop moduna gir ← seçilen
        1 = Deep sleep'te Standby moduna gir
    LPDS (bit 0): Low-Power Deep Sleep.
        0 = Stop'ta voltaj regülatörü normal modda (hızlı uyanma)
        1 = Stop'ta regülatör low-power modda (daha az akım, uyanma biraz yavaş) ← seçilen

SCB->SCR |= SLEEPDEEP:
    Cortex-M çekirdeğine "WFI çağrıldığında deep sleep'e gir" talimatı verir.
    PDDS=0 olduğundan deep sleep = Stop mode olur.

__WFI() sonrası kod akışı:
    Stop modunda program sayacı (PC) WFI instruction'ında bekler.
    EXTI interrupt'ı geldiğinde önce ilgili ISR çalışır, ardından
    WFI'dan sonraki satırdan devam edilir.

KRİTİK — Uyanma sonrası clock durumu:
    Stop'tan uyanıldığında sistem otomatik olarak HSI (16 MHz) ile çalışır!
    HSE ve PLL kapalıdır. SystemCoreClock artık 168 MHz DEĞİLDİR.
    Bu yüzden Clock_Init() çağrılarak HSE + PLL yeniden başlatılır.
    Bu yapılmazsa: UART baud rate'leri bozulur, delay'ler 10.5x yavaşlar,
    tüm timing hesapları geçersiz olur.

Uyandırma için EXTI gereksinimVERİFY:
    Stop'a girmeden önce bir EXTI line'ı yapılandırılmış ve NVIC'de
    etkin olmalıdır. Örn. GPIO_Interrupt_CMSIS modülündeki EXTI0 (PA0 butonu)
    veya RTC modülündeki Alarm A (EXTI line 17).
    Hiçbir uyandırma kaynağı yoksa MCU sonsuza kadar Stop'ta kalır —
    yalnızca reset ile çıkılabilir!

*/

/* LowPower_EnterStandby — Standby Mode
   En derin güç modu. 1.2V domain tamamen kapanır.
   SRAM ve register'lar SİLİNİR. Uyanma = RESET gibi davranır.
   Uyandırma: WKUP pini (PA0), RTC alarm, IWDG reset. */
void LowPower_EnterStandby(void){
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;                         // PWR clock enable
    (void)RCC->APB1ENR;

    PWR->CR |= PWR_CR_PDDS;                                     // PDDS = 1 → Standby mode
    PWR->CR |= PWR_CR_CWUF;                                     // Wakeup flag'ini temizle (WUF)
    while(PWR->CSR & PWR_CSR_WUF);                              // WUF temizlenene kadar bekle

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;                         // SLEEPDEEP = 1 → Deep sleep (Standby)

    __WFI();                                                     // Standby'a gir — buradan sonrası ÇALIŞMAZ

    /* Bu noktaya asla ulaşılmaz. Uyanma reset vektöründen başlar. */
}

/*

Amaç: MCU'yu Standby moduna almak (en derin güç tasarrufu, ~2 µA).

Standby Mode Genel Bilgi:
    1.2V domain tamamen kapatılır: CPU, SRAM, tüm register'lar güç kaybeder.
    Yalnızca backup domain (RTC, backup register'lar, LSE/LSI) çalışmaya devam eder.
    Uyanma bir RESET gibi davranır — program main()'in başından başlar.
    SRAM içeriği ve değişkenler KAYBOLUR; kalıcı veri backup register'larda
    (RTC->BKPxR) veya Flash'ta saklanmalıdır.

Uyandırma kaynakları:
    1. WKUP pini (PA0) yükselen kenar — PWR->CSR'de EWUP biti set edilmeli
    2. RTC alarm (Alarm A/B)
    3. RTC wakeup timer / tamper / timestamp olayı
    4. IWDG reset
    5. NRST pini (harici reset)

PWR->CR |= PWR_CR_CWUF ve while(WUF):
    Ne yapar: Wakeup flag'ini (WUF) temizler.
    Neden kritik: WUF set durumdayken Standby'a girilirse MCU ANINDA uyanır
    (önceki bir wakeup olayının bayrağı hâlâ aktif olduğundan).
    CWUF yazıldıktan sonra WUF'un gerçekten temizlendiğini beklemek gerekir
    (temizleme 2 sistem clock cycle sürer).

__WFI() sonrası:
    Standby'da SRAM silindiğinden WFI sonrası kod asla çalışmaz.
    Uyanma → reset vektörü → startup kodu → main() baştan.
    Uyanma sebebini anlamak için main() başında LowPower_WokeFromStandby()
    çağrılabilir.

*/

/* LowPower_WokeFromStandby — Standby'dan mı uyanıldı? */
uint8_t LowPower_WokeFromStandby(void){
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;                         // PWR clock enable
    (void)RCC->APB1ENR;

    if(PWR->CSR & PWR_CSR_SBF){                                 // Standby Flag set mi?
        PWR->CR |= PWR_CR_CSBF;                                 // SBF'yi temizle
        PWR->CR |= PWR_CR_CWUF;                                 // WUF'u da temizle
        return 1;                                                // Evet, Standby'dan uyanıldı
    }
    return 0;                                                    // Hayır, normal power-on/reset
}

/*

Amaç: Sistemin Standby modundan mı yoksa normal reset'ten mi başladığını ayırt etmek.

PWR->CSR & PWR_CSR_SBF (Standby Flag):
    Ne yapar: Sistem Standby modundan uyandıysa donanım bu bayrağı set eder.
    Normal power-on veya NRST reset'te bu bayrak 0'dır.

PWR->CR |= PWR_CR_CSBF:
    Ne yapar: SBF bayrağını temizler.
    Neden: Temizlenmezse bir sonraki normal reset'te de SBF set görünür
    ve yanlış değerlendirme yapılır.

Kullanım örneği (main başında):
    if(LowPower_WokeFromStandby()){
        // Standby'dan uyanıldı — özel başlatma yapılabilir
        // Örn: uyku sayacını backup register'dan oku
    } else {
        // Normal açılış — ilk kurulum yapılabilir
    }

*/

/* LowPower_ConfigWakeupPin — PA0 WKUP pin yapılandırması
   Standby'dan uyanmak için WKUP pinini etkinleştirir. */
void LowPower_ConfigWakeupPin(void){
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;                         // PWR clock enable
    (void)RCC->APB1ENR;

    PWR->CSR |= PWR_CSR_EWUP;                                   // Enable WKUP pin (PA0)
}

/*

Amaç: PA0 pinini Standby modundan uyandırma kaynağı olarak etkinleştirmek.

PWR->CSR |= PWR_CSR_EWUP (Enable Wakeup Pin):
    Ne yapar: PA0 pinini WKUP fonksiyonuna bağlar.
    EWUP = 1 iken PA0'da YÜKSELEN kenar algılanırsa Standby'dan uyanılır.
    Pin otomatik olarak input pull-down yapılandırmasına alınır
    (GPIO ayarı gerekmez — donanım devralır).

STM32F4 Discovery notu:
    User button (mavi buton) zaten PA0'a bağlıdır ve basıldığında
    HIGH olur (harici pull-down mevcut) — WKUP için idealdir.

DİKKAT:
    EWUP set edildikten sonra PA0 normal GPIO olarak KULLANILAMAZ;
    pin WKUP fonksiyonuna tahsis edilir. Standby kullanılmayacaksa
    EWUP temizlenmelidir.

*/
