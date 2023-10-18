#include <stm32f1xx.h>
#include "NRF24L01.hpp"
#include "tim_Delay.hpp"

#include <DigitalInterface/drivers.hpp>
#include <PortDriver/pin.hpp>
#include <Time&Sync/drivers.hpp>


int main(void)
{
PINx pin_mco(GPIOA,8);
PINx led_pin(GPIOC,13);
PINx CE_pin(GPIOA,4);
PINx IRQ_pin(GPIOA,2);

UARTLines uart_log{   USART1,2000000,
                      PINx(GPIOA,9),
                      PINx(GPIOA,10)};
 ClockSystem clock(pin_mco, uart_log);

  CE_pin.Settings(TYPE::OUTPUT_GPO_Push_Pull,LVL::LOW);
  led_pin.Settings(TYPE::OUTPUT_GPO_Push_Pull,LVL::LOW);
  IRQ_pin.Settings(TYPE::INPUT_Pull_Down,LVL::LOW);
 PINx spi_nrf24_mosi(GPIOA,7);
 PINx spi_nrf24_miso(GPIOA,6);
 PINx spi_nrf24_sck(GPIOA,5);
 PINx spi_nrf24_cs(GPIOA,3);

 SPILines nrf{ .SPIx=SPI1,
              .MOSI=spi_nrf24_mosi,
              .MISO=spi_nrf24_miso,
              .SCLK=spi_nrf24_sck,
              .NSS=spi_nrf24_cs,
              };
   SPI spi_nrf24L01(nrf);
spi_nrf24L01.SettingsSPI(
            RegCR1::ACTIVE,
            RegCR1::MASTER,
            2,
            RegCR1::SPI_MODE0,
            RegCR1::DFF8bit,
            RegCR1::MSBF,
            {},
            RegDMACR::TXDMA_DIS,
            RegDMACR::RXDMA_DIS);

static std::vector<uint8_t> RxData;
static std::vector<uint8_t> RxAddress{0xB3,0xB4,0xB5,0xB6,0xCD};
static std::vector<uint8_t> TxData{0x77,0x77,0x77};
static std::vector<uint8_t> TxAddress{0xB3,0xB4,0xB5,0xB6,0xCD};

uint8_t data[50];

NRF24_Init(spi_nrf24L01);
//NRF24_RxMode(spi_nrf24L01,RxAddress, 10);
//NRF24_TxMode(spi_nrf24L01,TxAddress, 10);
NRF24_TxMode(spi_nrf24L01,TxAddress, 10);
/*
nrf24_Write_Reg_multi(spi_nrf24L01,RX_ADDR_P0, std::vector<uint8_t>{0xE1, 0xE1, 0xE1, 0xE1, 0xE1});
*/
//nrf24_Read_Reg(spi_nrf24L01,EN_RXADDR,std::vector<uint8_t>(1,0));

 while(1)
 {
  //RX
/*
if (isDataAvailable(spi_nrf24L01,2) == 1)
	  {
      NRF24_Receive(spi_nrf24L01,RxData);
      led_pin.ToglePinLevel();
	  }*/

//TX
    
	  if (NRF24_Transmit(spi_nrf24L01,TxData) == 1)
	  {
      led_pin.ToglePinLevel();
	  }
   
 }
 }