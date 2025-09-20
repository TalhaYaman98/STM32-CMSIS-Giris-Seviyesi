#include "stm32f4xx.h"

void I2C1_Init(void)
{
    // RCC I2C1 ve GPIOB saatlerini aç
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;             // I2C1 çevresinin saatini etkinleştir [Binary: 0000 0000 0000 0000 0010 0000 0000 0000, Hex: 0x00200000] (APB1ENR register'ında 21. bit)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;            // GPIOB portunun saatini etkinleştir [Binary: 0000 0000 0000 0010, Hex: 0x00000002] (AHB1ENR register'ında 1. bit)

    // PB6 -> I2C1_SCL, PB7 -> I2C1_SDA
    GPIOB->MODER   &= ~(0xF << (6 * 2));              // PB6 ve PB7'nin MODER bitlerini temizle [Binary: 0000 0000 0000 1111 0000 0000 0000 0000, Hex: 0x0000F000] (PB6 ve PB7'nin 4 bitlik alanı temizlenir)
    GPIOB->MODER   |= (0xA << (6 * 2));               // PB6 ve PB7'yi alternatif fonksiyon moduna ayarla (AF mode) [Binary: 0000 0000 0000 1010 0000 0000 0000 0000, Hex: 0x0000A000] (PB6 ve PB7'yi AF moduna alır: PB6=10, PB7=10)
    GPIOB->OTYPER  |= (0x3 << 6);                     // PB6 ve PB7'yi open-drain yap [Binary: 0000 0000 0000 0000 0000 0000 1100 0000, Hex: 0x000000C0]
    GPIOB->OSPEEDR |= (0xF << (6 * 2));               // PB6 ve PB7'yi yüksek hız moduna ayarla [Binary: 0000 0000 0000 1111 0000 0000 0000 0000, Hex: 0x0000F000]
    GPIOB->PUPDR   |= (0x5 << (6 * 2));               // PB6 ve PB7'ye pull-up uygula [Binary: 0000 0000 0000 0101 0000 0000 0000 0000, Hex: 0x00005000]

    GPIOB->AFR[0] |= (0x44 << (6 * 4));             // PB6 ve PB7'yi AF4 (I2C1) olarak ayarla [Binary: 0100 0100 0000 0000 0000 0000 0000 0000, Hex: 0x44000000]

    // I2C1 reset
    I2C1->CR1 |= I2C_CR1_SWRST;                     // I2C1'i yazılımsal olarak resetle [Binary: 0000 0000 1000 0000, Hex: 0x0080] (CR1 register'ında 7. bit)
    I2C1->CR1 &= ~I2C_CR1_SWRST;                    // Reset bitini temizle [Aynı bit sıfırlanır]

    // Clock ayarı: 42 MHz APB1 için örnek.
    I2C1->CR2 = 42;                                 // APB1 saat frekansını MHz cinsinden ayarla [Binary: 0010 1010, Hex: 0x2A]
    I2C1->CCR = 210;                                // 100kHz için CCR değeri (CCR = Fpclk/(2*Fscl)) [Binary: 1101 0010, Hex: 0xD2]
    I2C1->TRISE = 43;                               // Maksimum yükselme süresi (TRISE = Fpclk(MHz) + 1) [Binary: 0010 1011, Hex: 0x2B]

    // I2C1 Enable
    I2C1->CR1 |= I2C_CR1_PE;                        // I2C1 çevresini etkinleştir [Binary: 0000 0000 0000 0001, Hex: 0x0001] (CR1 register'ında 0. bit)
}

void I2C1_Write(uint8_t slaveAddr, uint8_t *pData, uint32_t size)
{
    // START
    I2C1->CR1 |= I2C_CR1_START;                     // START koşulu oluştur [Binary: 0000 0000 0000 0001 0000, Hex: 0x0100]
    while (!(I2C1->SR1 & I2C_SR1_SB));              // START oluşana kadar bekle

    // Slave Address + Write
    I2C1->DR = slaveAddr << 1;                      // Slave adresini (yazma için) gönder [Binary: 0111 1000, Hex: 0x78]
    while (!(I2C1->SR1 & I2C_SR1_ADDR));            // Adres gönderimini bekle
    volatile uint32_t temp = I2C1->SR1 | I2C1->SR2; // ADDR bayrağını temizle (dummy read) [Binary: 0000 0000 0000 0010, Hex: 0x0002]

    // Data gönder
    for (uint32_t i = 0; i < size; i++)
    {
        while (!(I2C1->SR1 & I2C_SR1_TXE));         // Veri gönderilmeye hazır olana kadar bekle
        I2C1->DR = pData[i];                        // Veriyi gönder
    }

    while (!(I2C1->SR1 & I2C_SR1_BTF));             // Tüm byte'lar gönderilene kadar bekle (BTF: Byte Transfer Finished)

    // STOP
    I2C1->CR1 |= I2C_CR1_STOP;                      // STOP koşulu oluştur [Binary: 0000 0000 0001 0000 0000, Hex: 0x0200]
}

void I2C1_WriteByte(uint8_t slaveAddr, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2];                              // 2 byte'lık buffer oluştur
    buffer[0] = reg;                                // İlk byte: register adresi
    buffer[1] = data;                               // İkinci byte: veri
    I2C1_Write(slaveAddr, buffer, 2);               // Slave'e 2 byte gönder
}

void I2C1_ReadByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData)
{
    I2C1->CR1 |= I2C_CR1_START;                      // START koşulu oluştur
    while (!(I2C1->SR1 & I2C_SR1_SB));               // START oluşana kadar bekle
    I2C1->DR = slaveAddr << 1;                       // Slave adresini (yazma için) gönder
    while (!(I2C1->SR1 & I2C_SR1_ADDR));             // Adres gönderimini bekle
    volatile uint32_t temp = I2C1->SR1 | I2C1->SR2;  // ADDR bayrağını temizle (dummy read)

    while (!(I2C1->SR1 & I2C_SR1_TXE));              // Veri gönderilmeye hazır olana kadar bekle
    I2C1->DR = regAddr;                              // Okunacak register adresini gönder
    while (!(I2C1->SR1 & I2C_SR1_TXE));              // Adres gönderiminin tamamlanmasını bekle

    I2C1->CR1 |= I2C_CR1_START;                      // Tekrar START koşulu oluştur (tekrar başlatma)
    while (!(I2C1->SR1 & I2C_SR1_SB));               // START oluşana kadar bekle
    I2C1->DR = (slaveAddr << 1) | 0x01;              // Slave adresini (okuma için) gönder
    while (!(I2C1->SR1 & I2C_SR1_ADDR));             // Adres gönderimini bekle
    I2C1->CR1 &= ~I2C_CR1_ACK;                       // Son byte için ACK kapalı (NACK gönderilecek)
    temp = I2C1->SR1 | I2C1->SR2;                    // ADDR bayrağını temizle (dummy read)

    while (!(I2C1->SR1 & I2C_SR1_RXNE));             // Veri gelene kadar bekle
    *pData = I2C1->DR;                               // Okunan veriyi al

    I2C1->CR1 |= I2C_CR1_STOP;                       // STOP koşulu oluştur (haberleşmeyi sonlandır)
}

void I2C1_ReadBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint32_t size)
{
    I2C1->CR1 |= I2C_CR1_START;                      // START koşulu oluştur
    while (!(I2C1->SR1 & I2C_SR1_SB));               // START oluşana kadar bekle
    I2C1->DR = slaveAddr << 1;                       // Slave adresini (yazma için) gönder
    while (!(I2C1->SR1 & I2C_SR1_ADDR));             // Adres gönderimini bekle
    volatile uint32_t temp = I2C1->SR1 | I2C1->SR2;  // ADDR bayrağını temizle (dummy read)

    while (!(I2C1->SR1 & I2C_SR1_TXE));              // Veri gönderilmeye hazır olana kadar bekle
    I2C1->DR = regAddr;                              // Okunacak register adresini gönder
    while (!(I2C1->SR1 & I2C_SR1_TXE));              // Adres gönderiminin tamamlanmasını bekle

    I2C1->CR1 |= I2C_CR1_START;                      // Tekrar START koşulu oluştur (tekrar başlatma)
    while (!(I2C1->SR1 & I2C_SR1_SB));               // START oluşana kadar bekle
    I2C1->DR = (slaveAddr << 1) | 0x01;              // Slave adresini (okuma için) gönder
    while (!(I2C1->SR1 & I2C_SR1_ADDR));             // Adres gönderimini bekle
    temp = I2C1->SR1 | I2C1->SR2;                    // ADDR bayrağını temizle (dummy read)

    I2C1->CR1 |= I2C_CR1_ACK;                        // ACK açık (son byte hariç)

    for (uint32_t i = 0; i < size; i++)
    {
        if (i == size - 1)
        {
            I2C1->CR1 &= ~I2C_CR1_ACK;               // Son byte için ACK kapalı (NACK gönderilecek)
        }
        while (!(I2C1->SR1 & I2C_SR1_RXNE));         // Veri gelene kadar bekle
        pData[i] = I2C1->DR;                         // Okunan veriyi al
    }

    I2C1->CR1 |= I2C_CR1_STOP;                       // STOP koşulu oluştur (haberleşmeyi sonlandır)
}


/*

STM32F4’te I2C register yapıları:

	CR1			Kontrol register 1 (I2C’yi başlatma, durdurma, reset, enable, ACK vb.)
	CR2			Kontrol register 2 (frekans ayarı, interrupt enable vb.)
	OAR1, OAR2	Own Address Register (slave adresi tanımlama)
	DR			Data Register (veri gönderme/alma)
	SR1			Status Register 1 (bayraklar: SB, ADDR, BTF, RXNE, TXE, STOPF, vb.)
	SR2			Status Register 2 (busy, master/slave, genel durum)
	CCR			Clock Control Register (I2C clock ayarı)
	TRISE		Maksimum yükselme süresi ayarı
	FLTR		Filtre ayarları (opsiyonel)

Sık Kullanılan Register ve Bitler

	CR1 (Control Register 1)
		PE : I2C çevresini enable/disable eder.
		START : Start koşulu üretir.
		STOP : Stop koşulu üretir.
		ACK : ACK/NACK kontrolü.
		SWRST : Yazılımsal reset.

	CR2 (Control Register 2)
		FREQ[5:0]: APB1 clock frekansı (MHz cinsinden).
		ITEVTEN, ITBUFEN, ITERREN: Interrupt enable bitleri.

	SR1 (Status Register 1)
		SB : Start bit set (start koşulu oluştu).
		ADDR : Adres gönderildi/alındı.
		BTF : Byte transfer finished.
		RXNE : Data register not empty (alıcı için).
		TXE : Data register empty (gönderici için).
		STOPF : Stop detected.

	SR2 (Status Register 2)
		BUSY : I2C hattı meşgul mü?
		MSL : Master/slave durumu.
		TRA : Transmitter/receiver durumu.

	DR (Data Register)
		Veri gönderme/alma için kullanılır.
		CCR (Clock Control Register)
		I2C clock hızını ayarlamak için kullanılır.

	TRISE
		Maksimum yükselme süresi ayarı (I2C standardına göre).

Basit bir I2C transmit (yazma) ve receive (okuma) işlemi için mutlaka kullanılması gereken temel registerlar ve doğru sıralama aşağıdaki gibidir:

1. I2C'yi Başlatmak İçin Gerekli Registerlar ve Sıralama

	Saat Sinyali Açılır
		RCC->APB1ENR (I2C clock enable)
		RCC->AHB1ENR (GPIO clock enable)

	GPIO Pinleri I2C için Ayarlanır
		GPIOx->MODER (AF mode)
		GPIOx->OTYPER (open-drain)
		GPIOx->OSPEEDR (hız)
		GPIOx->PUPDR (pull-up)
		GPIOx->AFR[] (AF4/I2C)

	I2C Zamanlama ve Mod Ayarları
		I2C1->CR2 (APB1 frekansı)
		I2C1->CCR (clock control)
		I2C1->TRISE (rise time)

	I2C Enable
		I2C1->CR1 (PE biti)

2. Transmit (Yazma) ve Receive (Okuma) İçin Gerekli Registerlar ve Sıralama

	Transmit (Yazma) Sırası

		START koşulu:
			I2C1->CR1 (START biti set edilir)
		START oluştu mu kontrol:
			I2C1->SR1 (SB bayrağı)
		Slave adresi gönder:
			I2C1->DR (slave adresi yazılır)
		Slave cevapladı mı kontrol:
			I2C1->SR1 (ADDR bayrağı)
			I2C1->SR2 (ADDR bayrağını temizlemek için okunur)
		Veri gönder:
			I2C1->DR (veri yazılır)
			I2C1->SR1 (TXE ve BTF bayrakları kontrol edilir)
		STOP koşulu:
			I2C1->CR1 (STOP biti set edilir)

	Receive (Okuma) Sırası

		START koşulu:
			I2C1->CR1 (START biti set edilir)
		START oluştu mu kontrol:
			I2C1->SR1 (SB bayrağı)
		Slave adresi (read) gönder:
			I2C1->DR (slave adresi + 1 yazılır)
		Slave cevapladı mı kontrol:
			I2C1->SR1 (ADDR bayrağı)
			I2C1->SR2 (ADDR bayrağını temizlemek için okunur)
		ACK/NACK ayarı:
			I2C1->CR1 (ACK biti, son byte için NACK yapılır)
		Veri oku:
			I2C1->SR1 (RXNE bayrağı kontrol edilir)
			I2C1->DR (veri okunur)
		STOP koşulu:
			I2C1->CR1 (STOP biti set edilir)

Özetle Gerekli Registerlar
RCC->APB1ENR, RCC->AHB1ENR (clock enable)
GPIOx->MODER, OTYPER, OSPEEDR, PUPDR, AFR[] (pin ayarları)
I2C1->CR2, CCR, TRISE (zamanlama)
I2C1->CR1 (enable, start, stop, ack)
I2C1->SR1, SR2 (bayraklar)
I2C1->DR (veri gönder/al)

Bu temel registerlar ve sıralama, basit bir I2C haberleşmesi için yeterlidir.
Daha ileri özellikler (interrupt, DMA, error handling) için ek registerlar kullanılabilir.

Tek byte okuma ve çok byte okuma için ayrı fonksiyonlar yazmamızın nedeni, I2C protokolünde ve sensörün (örneğin MPU6050) veri yapısında okuma işlemlerinin farklı şekilde yönetilmesidir.
	Tek Byte Okuma
		Sadece bir register’dan (örneğin WHO_AM_I → adres: 0x75) tek bir veri byte’ı okumak için kullanılır.
		Okuma işlemi sonunda hemen STOP gönderilir.
		Genellikle cihaz kimliği, durum bayrağı gibi tek byte’lık veri için uygundur.
	Çok Byte Okuma
		MPU6050’de ivme, gyro, sıcaklık gibi veriler birden fazla byte’lık register’larda ardışık olarak tutulur (örneğin ACCEL_XOUT_H ve devamı).
		Çoklu okuma fonksiyonu, bir başlangıç adresinden itibaren birden fazla byte’ı ardışık olarak okur.
		Her byte okunduktan sonra son byte için NACK (ACK kapalı) gönderilir ve STOP ile bitirilir.
		Sensörün bir eksenini veya tüm eksenlerini tek seferde almak için gereklidir.
	Neden Ayrı Fonksiyon?
		I2C protokolünde son byte okuması sırasında ACK/NACK yönetimi farklıdır.
		Tek byte okuma işlemi daha basittir, çoklu okuma ise döngü ve son byte için özel işlem gerektirir.
		Sensörün veri yapısına uygun şekilde, ihtiyaca göre fonksiyon seçilir.
	Özet:
		MPU6050 gibi sensörlerde, tek byte okuma ile cihaz kimliği veya bir ayar okunur; çok byte okuma ile sensör verileri (ivme, gyro vs.) topluca okunur.
		Her iki işlem için I2C protokolünde farklı adımlar ve bit yönetimi gerektiğinden ayrı fonksiyonlar kullanılır.

*/
