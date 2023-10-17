#include <stm32f1xx.h>
#include "NRF24L01.hpp"
#include "tim_Delay.hpp"

#include <DigitalInterface/drivers.hpp>
#include <PortDriver/pin.hpp>
#include <Time&Sync/drivers.hpp>

/*
 static std::array<uint8_t,5> data;
 uint8_t *ptr;

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

void nrf24_Read_Reg(uint8_t reg,uint8_t size)
{
  std::vector<uint8_t> Buffer_rx;
 Buffer_rx.resize(size+1);
 Buffer_rx.at(0)=reg|R_REGISTER;
 spi_nrf24L01.Recieve(Buffer_rx);
 ptr= reinterpret_cast<uint8_t*>(data.data());
 volatile uint8_t size2=Buffer_rx.size();
 for (uint8_t i=0;i<size2;i++)
 {
  data.at(i)=Buffer_rx.at(i);
 }


 Buffer_rx.at(0)=0;
}
void nrf24_Write_Reg(uint8_t reg,uint8_t value)
{
 static std::vector<uint8_t> Buffer_tx;
 Buffer_tx.clear();
 Buffer_tx.resize(2);
 Buffer_tx.at(0)=reg|W_REGISTER;
 Buffer_tx.at(1)=value;
 spi_nrf24L01.Transmitt(Buffer_tx);
}
*/


int main(void)
{


PINx pin_mco(GPIOA,8);
UARTLines uart_log{   USART1,2000000,
                      PINx(GPIOA,9),
                      PINx(GPIOA,10)};
 ClockSystem clock(pin_mco, uart_log);

	PINx CE_pin(GPIOA,4);
    CE_pin.Settings(TYPE::OUTPUT_GPO_Push_Pull,LVL::LOW);


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


NRF24_Init(spi_nrf24L01);

/*
nrf24_Write_Reg_multi(spi_nrf24L01,RX_ADDR_P0, std::vector<uint8_t>{0xE1, 0xE1, 0xE1, 0xE1, 0xE1});
nrf24_Read_Reg(spi_nrf24L01,RX_ADDR_P0,std::vector<uint8_t>(5,0));
*/
uint16_t k=0;
k++;

 while(1)
 {

/*
CE_pin.SetPinLevel(LVL::HIGH);
CE_pin.SetPinLevel(LVL::LOW);*/
 }
 }