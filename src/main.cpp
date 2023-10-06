#include <stm32f1xx.h>
#include "NRF24L01.hpp"
#include "tim_Delay.hpp"

#include <DigitalInterface/drivers.hpp>
#include <Time&Sync/drivers.hpp>

 std::array<uint8_t,5> data;
uint8_t *ptr;
int main(void)
{
 //SetSysClockTo72();
 //gpio_init();
 //spi_init();

/*
PINx spi_nrf24_mosi(GPIOA,5);
PINx spi_nrf24_miso(GPIOA,4);
PINx spi_nrf24_sck(GPIOA,3);
PINx spi_nrf24_cs(GPIOA,2);
#define SPI1_CE    4
#define SPI1_NSS   3
#define SPI1_SCK   5
#define SPI1_MISO  6
#define SPI1_MOSI  7*/
PINx pin_mco(GPIOA,8);
UARTLines uart_log{   USART1,2000000,
                      PINx(GPIOA,9),
                      PINx(GPIOA,10)};
ClockSystem clock(pin_mco, uart_log);

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

/*UART debug(UART1,
                      baremetal.BaudRate,
                      std::make_unique<PINx>(baremetal.TxD),
                      std::make_unique<PINx>(baremetal.RxD));*/

 static std::vector<uint8_t> Packet{};
 Packet.clear();
 Packet.resize(2);
 Packet.at(0)=CONFIG|W_REGISTER;
 Packet.at(1)=0x08;
 spi_nrf24L01.Transmitt(Packet);
 data.at(0)=Packet.at(0);
 Packet.resize(2);
 Packet.at(0)=CONFIG|R_REGISTER;
 Packet.at(1)=0x00;
 spi_nrf24L01.Recieve(Packet);
ptr= reinterpret_cast<uint8_t*>(Packet.data());

data.at(0)=Packet.at(0);
data.at(1)=Packet.at(1);
/*
NRF24_Init();
*/
 while(1)
 {
 Packet.at(1)=0x08;
/*
  data[0]=nrf24_ReadReg(CONFIG);
  data[1]=nrf24_ReadReg(EN_AA);
  data[2]=nrf24_ReadReg(EN_RXADDR);
  data[3]=nrf24_ReadReg(RX_ADDR_P1);
  data[4]=nrf24_ReadReg(RX_ADDR_P2);
  data[5]=nrf24_ReadReg(RX_ADDR_P3);
  data[6]=nrf24_ReadReg(RX_ADDR_P4);
  data[7]=nrf24_ReadReg(RX_ADDR_P5);
  
data[30]=55+56;*/


 }
 }