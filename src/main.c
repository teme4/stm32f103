#include <stm32f1xx.h>
#include "hardware_config.h"
#include "gpio.h"
#include "NRF24L01.h"

uint8_t spi_transmit(uint16_t data)
{
while (!(SPI1->SR & SPI_SR_TXE));
SPI1->DR = data;
while(SPI1->SR & SPI_SR_BSY) {}
while(!(SPI1->SR & SPI_SR_RXNE));// Ждём получения данных, читаем их.
data=SPI1->DR;
return data;
}


/*
void spi_transmit_multi(uint16_t *data)
{
while (!(SPI1->SR & SPI_SR_TXE));
SPI1->DR = data;
while(!(SPI1->SR&SPI_SR_RXNE)) {}// Ждём получения данных, читаем их.
data=SPI1->DR;
return data;
}*/

void spi_recive(uint16_t data)
{
while(!(SPI1->SR&SPI_SR_RXNE)) {}
data=SPI1->DR;
return data;
}





void spi_init()
{
 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; //--- Включаем тактирование альтернативных функции
 RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //--- Включаем тактированние SPI1 
// Указываем полярность и фазу тактового сигнала SPI.
const uint32_t CPOL=SPI_CR1_CPOL*0;
const uint32_t CPHA=SPI_CR1_CPHA*0;

// Конфигурируем SPI1 (обычный ведущий режим в данном случае).
    // BIDIMODE: 0 (выбор режима с одной линией данных - отключено);
    // BIDIOE: 0 (направление передачи, бит используется при BIDIMODE=1);
    // CRCEN: 0 (аппаратный подсчёт CRC отключён);
    // CRCNEXT:0 (отправка CRC, используется при CRCEN=1;
    // DFF: 0 (длина фрейма данных, здесь - 8-битовый фрейм);
    // RXONLY: 0 (включение режима "только приём", здесь - полнодуплекс);
    // SSM: 0 (включение режима программного управления сигналом NSS);
    // SSI: 0 (значение бита используется вместо сигнала NSS при SSM=1);
    // LSBFIRST: 0 (порядок передачи битов, здесь - первым передаётся старший);
    // SPE: 0 (бит включения SPI, здесь разделяем конфигурирование и включение);
    // BR[2:0] (управление скоростью передачи, только для ведущего;
            // здесь задаём 0x7, что соотв. макс. делителю /256 и мин. скорости);
    // MSTR: 1 (бит переключения в ведущий режим).
    SPI1->CR1=              // Большинство битов задаём нулевыми.
            SPI_CR1_MSTR|   // Ведущий режим.
            SPI_CR1_BR_2|     // BR[2:0]=0x7 - минимальная скорость для теста.
            CPOL|           // Полярность тактового сигнала.
            CPHA;           // Фаза тактового сигнала.
        SPI1->CR1 &= ~SPI_CR1_DFF;
  /*
  SPI1->CR1 |= 1<<2; //режим мастера  MSTR
	SPI1->CR1 |= 0b111<<3; //скорость 000 f/2,001 f/4,...,111 f/256   BR [2:0]
	SPI1->CR1 |= 1<<8; //SSI внутреннее управление слейвом
	SPI1->CR1 |= 1<<9; //SSM програмное управление слейвом*/

    // С помощью регистра CR2 настраиваем генерацию запросов на
    // прерывание и DMA (если нужно); с помощью бита SSOE запрещаем или
    // разрешаем использовать ведущему устройству вывод NSS как выход.
    SPI1->CR2&=~0xE7;       // Сбрасываем все значимые биты регистра.
    SPI1->CR2|=SPI_CR2_SSOE;// NSS будет выходом.

    // Включаем SPI1.
    // Передача не начнётся, пока не запишем что-то в его регистр,
    // данных, но установится состояние выходов.
    SPI1->CR1|=SPI_CR1_SPE;

}


int main(void)
{
 //SetSysClockTo72();
 gpio_init();
 spi_init();
NRF24_Init();
volatile uint8_t data=0;
volatile uint8_t buf_in[32];
 while(1)
 {

/*
nrf24_ReadReg_Multi(RX_ADDR_P0, buf_in, 5);
 data=nrf24_ReadReg(RX_ADDR_P0);
*/

  data=nrf24_ReadReg(CONFIG);
  data=0;
  data=nrf24_ReadReg(EN_AA);
  data=0;
  data=nrf24_ReadReg(EN_RXADDR);
  data=0;
  data=0;
  data=nrf24_ReadReg(RX_ADDR_P1);
  data=0;
  data=nrf24_ReadReg(RX_ADDR_P2);
  data=0;
  data=nrf24_ReadReg(RX_ADDR_P3);
  data=0;
  data=nrf24_ReadReg(RX_ADDR_P4);
  data=0;
  data=nrf24_ReadReg(RX_ADDR_P5);
  data=0;



 }
 }