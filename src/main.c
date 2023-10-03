#include <stm32f1xx.h>
#include "hardware_config.h"
#include "gpio.h"
#include "NRF24L01.h"

// clang-format off
enum class IRQnSPI
{
    TXEIE                                       = (1<<7),///<прерываниe по переполнени. приемного буфера FIFO
    RXNEIE                                      = (1<<6),///<прерываниe по таймауту приемника (буфер FIFO приемника не пуст и не было попыток его чтения в течение времени таймаута)
    ERRIE                                       = (1<<5),///<прерываниe по заполнению на 50 % и более буфера FIFO приемника
    NONE                                        = (0<<0)
};

enum class RegCR1
{
    SPI_MODE0                                   = (0b00 ),///<SPI фирмы Motorola(CPOL = 0, CPHA = 0);
    SPI_MODE1                                   = (0b01 ),///<SPI фирмы Motorola(CPOL = 0, CPHA = 1);
    SPI_MODE2                                   = (0b10 ),///<SPI фирмы Motorola(CPOL = 1, CPHA = 0);
    SPI_MODE3                                   = (0b11 ),///<SPI фирмы Motorola(CPOL = 1, CPHA = 1);
    MASTER                                      = (1<<2 ),///<ведущий модуль
    SLAVE                                       = (0<<0 ),///<ведомый модуль
    ACTIVE                                      = (1<<6 ),///<работа разрешена
    INACTIVE                                    = (0<<0 ),///<работа запрещена;
    DFF8bit                                     = (0<<0 ),///<data frame format: 8bit;
    DFF16bit                                    = (1<<11),///<data frame format: 16bit;
    LSBF                                        = (1<<7 ),///<LSB  transmitted first
    MSBF                                        = (0<<0 ),///<MSB transmitted first
};
enum class RegDMACR
{
    RXDMA_DIS                                   = 0x00,///<Запрещено формирование запросов DMA буфера FIFO приемника
    RXDMA_EN                                    = 0x01,///<Разрешено формирование запросов DMA буфера FIFO приемника
    TXDMA_DIS                                   = 0x00,///<Запрещено формирование запросов DMA буфера FIFO передатчика
    TXDMA_EN                                    = 0x02,///<Разрешено формирование запросов DMA буфера FIFO передатчика
    NONE                                        = 0x00
};

void SettingsSPI (SPI_TypeDef*SPIx ,RegCR1 SPE,
                      RegCR1 MS,
                      double frequency,
                      RegCR1 Type,
                      RegCR1 WordSize,
                      RegCR1 LsbMsbFirst)
                      {
   RCC->APB1ENR |= RCC_APB1ENR_SPI2EN  ; // ??? ???????????? SPI

    SPIx->CR1    = 0;
    SPIx->CR2    = 0;
    SPIx->SR     = 0;
    SPIx->CRCPR  = 0;
    SPIx->RXCRCR = 0;
    SPIx->TXCRCR = 0;
    //BRR = static_cast<uint8_t>(log2(F_SPICLK / (Frequency * 1000000)) - 1);

    SPIx->CR1 |=  SPI_CR1_BR;
    SPIx->CR1 |= static_cast<uint32_t>(Type);
    SPIx->CR1 |= static_cast<uint32_t>(WordSize);
    SPIx->CR1 |= static_cast<uint32_t>(LsbMsbFirst);
    SPIx->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;
    SPIx->CR1 |= static_cast<uint32_t>(MS);

   // SPIx->CR2 |= static_cast<uint32_t>(TxDmacr);
    //SPIx->CR2 |= static_cast<uint32_t>(RxDmacr);

    SPIx->CR1 |= static_cast<uint32_t>(SPE);
                      }

void spi_transmit(uint16_t data)
{
while (!(SPI2->SR & SPI_SR_TXE));
SPI2->DR = data ;
}




int main(void) 
{
 gpio_init();

 while(1) 
 {

 }
 }