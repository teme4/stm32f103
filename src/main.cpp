#include <stm32f1xx.h>
#include "NRF24L01.hpp"

#include <stdio.h>
#include <DigitalInterface/drivers.hpp>
#include <PortDriver/pin.hpp>
#include <Time&Sync/drivers.hpp>


int main(void)
{
PINx pin_mco(GPIOA,8);
PINx led_pin(GPIOC,13);
PINx CE_pin(GPIOA,4);
PINx IRQ_pin(GPIOA,2);

UARTLines uart_log{USART1,256000,
                      PINx(GPIOA,9),//TxD
                      PINx(GPIOA,10)};//RxD

  //ClockSystem clock(pin_mco, uart_log);
UART uart__1(uart_log);
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
//static std::vector<uint8_t> RxAddress{0xB3,0xB4,0xB5,0xB6,0xCD};
static std::vector<uint8_t> TxData{0x77,0x77,0x77};
//static std::vector<uint8_t> TxAddress{0xB3,0xB4,0xB5,0xB6,0xCD};

static std::vector<uint8_t> RxAddress{49,78,111,100,101};
static std::vector<uint8_t> TxAddress{49,78,111,100,101};

uint8_t data[50];

  begin(spi_nrf24L01);
  setAutoAck(spi_nrf24L01,false);
  startListening(spi_nrf24L01);
  //printDetails(spi_nrf24L01);

//NRF24_RxMode(spi_nrf24L01,RxAddress, 15);
//NRF24_TxMode2(spi_nrf24L01,TxAddress,15);
//setChannel(spi_nrf24L01,15);
//setPALevel(spi_nrf24L01, RF24_PA_MAX);
 /*
char uart_buff[100];
strcpy(uart_buff, "SETUP_AW");
 uart_transsmite_text(uart_buff,strlen(uart_buff));
 nrf24_Read_Reg(spi_nrf24L01,RX_ADDR_P2,std::vector<uint8_t>(1,0));
 strcpy(uart_buff, "RF_SETUP");
 uart_transsmite_text(uart_buff,strlen(uart_buff));
 nrf24_Read_Reg(spi_nrf24L01,RX_ADDR_P3,std::vector<uint8_t>(1,0));*/

 //uart_transsmite_text("RX_ADDR_P2");
 //nrf24_Read_Reg(spi_nrf24L01,RX_ADDR_P2,std::vector<uint8_t>(1,0));
 //MESSAGE_INFO(" Functional Control : Started\n");
//uart__1.Transmitt();


//NRF24_TxMode(spi_nrf24L01,TxAddress, 100);
/*
nrf24_Write_Reg_multi(spi_nrf24L01,RX_ADDR_P0, std::vector<uint8_t>{0xE1, 0xE1, 0xE1, 0xE1, 0xE1});
*/
//nrf24_Read_Reg(spi_nrf24L01,EN_RXADDR,std::vector<uint8_t>(1,0));

 while(1)
 {
  //RX5
/*
if (isDataAvailable(spi_nrf24L01,2) == 1)
	  {
      NRF24_Receive(spi_nrf24L01,RxData);
      led_pin.ToglePinLevel();
	  }
*/
//TX
get_status(spi_nrf24L01);
  
  /*  if (NRF24_Transmit(spi_nrf24L01,TxData) == 1)
	  {
      led_pin.ToglePinLevel();
	  }*/



    /*
 nrf24_Read_Reg(spi_nrf24L01,CONFIG,std::vector<uint8_t>(1,0));
  nrf24_Read_Reg(spi_nrf24L01,CONFIG,std::vector<uint8_t>(1,0));
   nrf24_Read_Reg(spi_nrf24L01,CONFIG,std::vector<uint8_t>(1,0));
    nrf24_Read_Reg(spi_nrf24L01,CONFIG,std::vector<uint8_t>(1,0));
   led_pin.ToglePinLevel();*/
  }
 }