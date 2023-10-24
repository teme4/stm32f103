/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  NRF24L01.c
  Author:     ControllersTech.com
  Updated:    30th APRIL 2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/

#include "NRF24L01.hpp"
#include <stm32f1xx.h>

 static std::array<uint8_t,32> data;
 uint8_t *ptr;
 extern PINx CE_pin(GPIOA,4);
 extern PINx led_pin(GPIOC,13);
extern PINx IRQ_pin(GPIOA,2);
//******************************************************************//
void nrf24_Read_Reg(SPI& spi_nrf24L01,uint8_t reg,std::vector<uint8_t> Buffer_rx)
{
 Buffer_rx.reserve(1);
 Buffer_rx.insert(Buffer_rx.begin(),reg|R_REGISTER);
 volatile uint8_t size2=Buffer_rx.size();
 uint8_t temp1=0,temp2=0;
 spi_nrf24L01.Recieve(Buffer_rx);
 ptr= reinterpret_cast<uint8_t*>(data.data());
 size2=Buffer_rx.size();
 temp1=Buffer_rx.at(0);
 temp2=Buffer_rx.at(1);
 for (uint8_t i=0;i<size2;i++)
 {
   data.at(i)=Buffer_rx.at(i);
 }
 size2=Buffer_rx.size();
 }
//******************************************************************//
void nrf24_Write_Reg(SPI& spi_nrf24L01,uint8_t reg,uint8_t value)
{
 static std::vector<uint8_t> Buffer_tx;
 Buffer_tx.clear();
 Buffer_tx.resize(2);
 Buffer_tx.at(0)=reg|W_REGISTER;
 Buffer_tx.at(1)=value;
 uint8_t temp1= Buffer_tx.at(0);
uint8_t temp2= Buffer_tx.at(1);
 spi_nrf24L01.Transmitt(Buffer_tx);
}
//******************************************************************//
void nrf24_Write_Reg_multi(SPI& spi_nrf24L01,uint8_t reg,std::vector<uint8_t> Buffer_tx)
{
 Buffer_tx.reserve(1);
 Buffer_tx.insert(Buffer_tx.begin(),reg|W_REGISTER);
 spi_nrf24L01.Transmitt(Buffer_tx);
}
//******************************************************************//
// send the command to the NRF
void nrfsendCmd (SPI& spi_nrf24L01,std::vector<uint8_t> Buffer_tx)
{
	spi_nrf24L01.Transmitt(Buffer_tx);
}
//******************************************************************//
uint8_t nrf24_reset(SPI& spi_nrf24L01,uint8_t REG)
{
	volatile uint8_t check=0;
	if (REG == STATUS)
	{
		nrf24_Write_Reg(spi_nrf24L01,STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_Write_Reg(spi_nrf24L01,FIFO_STATUS, 0x11);
	}
	else
	{
	nrf24_Write_Reg(spi_nrf24L01,CONFIG, 0x08);
	//check=nrf24_ReadReg(CONFIG);
	nrf24_Read_Reg(spi_nrf24L01,CONFIG,std::vector<uint8_t>(1,0));
	check=data.at(1);
	if(check!=0x08)
	{
		return 0;
	}
	nrf24_Write_Reg(spi_nrf24L01,EN_AA, 0x3F);
	nrf24_Write_Reg(spi_nrf24L01,EN_RXADDR, 0x02);
	nrf24_Write_Reg(spi_nrf24L01,SETUP_AW, 0x03);
	nrf24_Write_Reg(spi_nrf24L01,SETUP_RETR, 0x03);
	nrf24_Write_Reg(spi_nrf24L01,RF_CH, 0x02);
	nrf24_Write_Reg(spi_nrf24L01,RF_SETUP, 0x0C);
	nrf24_Write_Reg(spi_nrf24L01,STATUS, 0x00);
	nrf24_Write_Reg(spi_nrf24L01,OBSERVE_TX, 0x00);
	nrf24_Write_Reg(spi_nrf24L01,CD, 0x00);
	nrf24_Write_Reg_multi(spi_nrf24L01,RX_ADDR_P0, std::vector<uint8_t>{0xE7, 0xE7, 0xE7, 0xE7, 0xE7});
	nrf24_Write_Reg_multi(spi_nrf24L01,RX_ADDR_P1, std::vector<uint8_t>{0xC2, 0xC2, 0xC2, 0xC2, 0xC2});
	nrf24_Write_Reg(spi_nrf24L01,RX_ADDR_P2, 0xC3);
	nrf24_Write_Reg(spi_nrf24L01,RX_ADDR_P3, 0xC4);
	nrf24_Write_Reg(spi_nrf24L01,RX_ADDR_P4, 0xC5);
	nrf24_Write_Reg(spi_nrf24L01,RX_ADDR_P5, 0xC6);
	nrf24_Write_Reg_multi(spi_nrf24L01,TX_ADDR, std::vector<uint8_t>{0xE7, 0xE7, 0xE7, 0xE7, 0xE7});
	nrf24_Write_Reg(spi_nrf24L01,RX_PW_P0, 0);
	nrf24_Write_Reg(spi_nrf24L01,RX_PW_P1, 0);
	nrf24_Write_Reg(spi_nrf24L01,RX_PW_P2, 0);
	nrf24_Write_Reg(spi_nrf24L01,RX_PW_P3, 0);
	nrf24_Write_Reg(spi_nrf24L01,RX_PW_P4, 0);
	nrf24_Write_Reg(spi_nrf24L01,RX_PW_P5, 0);
	nrf24_Write_Reg(spi_nrf24L01,FIFO_STATUS, 0x11);
	nrf24_Write_Reg(spi_nrf24L01,DYNPD, 0);
	nrf24_Write_Reg(spi_nrf24L01,FEATURE, 0);
	}
}

void NRF24_Init (SPI& spi_nrf24L01)
{
	CE_pin.SetPinLevel(LVL::LOW);
	nrf24_reset (spi_nrf24L01,0);
	nrf24_Write_Reg(spi_nrf24L01,CONFIG, 0);  // will be configured later
	nrf24_Write_Reg(spi_nrf24L01,EN_AA, 0);  // No Auto ACK
	nrf24_Write_Reg (spi_nrf24L01,EN_RXADDR, 0);  // Not Enabling any data pipe right now
	nrf24_Write_Reg (spi_nrf24L01,SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address
	nrf24_Write_Reg (spi_nrf24L01,SETUP_RETR, 0);   // No retransmission
	nrf24_Write_Reg (spi_nrf24L01,RF_CH, 0);  // will be setup during Tx or RX
	nrf24_Write_Reg (spi_nrf24L01,RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps
	CE_pin.SetPinLevel(LVL::HIGH);
}

// set up the Tx mode

void NRF24_TxMode (SPI& spi_nrf24L01,std::vector<uint8_t> Address, uint8_t channel)
{
	CE_pin.SetPinLevel(LVL::LOW);
	nrf24_Write_Reg (spi_nrf24L01,RF_CH, channel);  // select the channel
	nrf24_Write_Reg_multi(spi_nrf24L01,TX_ADDR,Address);
	// power up the device
	nrf24_Read_Reg(spi_nrf24L01,CONFIG,std::vector<uint8_t>(1,0));
	uint8_t  config=data.at(1);
	//config = config | (1<<1);   // write 1 in the PWR_UP bit
	config = config & (0xF2);    // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
	nrf24_Write_Reg (spi_nrf24L01,CONFIG, config);
	CE_pin.SetPinLevel(LVL::HIGH);
}

// transmit the data

uint8_t NRF24_Transmit (SPI& spi_nrf24L01,std::vector<uint8_t> data)
{
    nrf24_Write_Reg_multi(spi_nrf24L01,W_TX_PAYLOAD, data);
	//uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	/* for(int i=1;i<99999;i++) //HAL_Delay(1);
    {
      int k=i+5-3;
    }*/
		/*while (1)//Ждем пока байт не передан
	{
		if(IRQ_pin.GetPinLevel()==0)
		break;
	}*/
    nrf24_Read_Reg(spi_nrf24L01,FIFO_STATUS,std::vector<uint8_t>(1,0));
	uint8_t fifostatus =data.at(1);
	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
	if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
	{
		//cmdtosend = FLUSH_TX;
		static std::vector<uint8_t> Buffer_tx;
		Buffer_tx.reserve(1);
 		Buffer_tx.insert(Buffer_tx.begin(),FLUSH_TX);
		nrfsendCmd(spi_nrf24L01,Buffer_tx);
		// reset FIFO_STATUS
		nrf24_reset (spi_nrf24L01,FIFO_STATUS);
		return 1;
	}
	return 0;
}

void delay()
{
	for(int i=0;i<99999;i++)
	{
		i=i++;
	}
}

void NRF24_RxMode2 (SPI& spi_nrf24L01, std::vector<uint8_t> Address, uint8_t channel)
{
CE_pin.SetPinLevel(LVL::LOW);
nrf24_Write_Reg (spi_nrf24L01,CONFIG,(1<<PWR_UP)|(1<<EN_CRC)|(0<<PRIM_RX));
delay();
nrf24_Write_Reg (spi_nrf24L01,CONFIG,(1<<PWR_UP)|(1<<EN_CRC)|(1<<PRIM_RX));
CE_pin.SetPinLevel(LVL::HIGH);
delay();
}

void NRF24_TxMode2 (SPI& spi_nrf24L01,std::vector<uint8_t> Address, uint8_t channel)
{
CE_pin.SetPinLevel(LVL::LOW);
nrf24_Write_Reg (spi_nrf24L01,CONFIG,(1<<PWR_UP)|(1<<EN_CRC)|(0<<PRIM_RX));
delay();
}

void NRF24_RxMode (SPI& spi_nrf24L01, std::vector<uint8_t> Address, uint8_t channel)
{
	// disable the chip before configuring the device
	CE_pin.SetPinLevel(LVL::LOW);
	nrf24_reset (spi_nrf24L01,STATUS);
	nrf24_Write_Reg (spi_nrf24L01,RF_CH, channel);  // select the channel

	// select data pipe 2
	//unt8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	nrf24_Read_Reg(spi_nrf24L01,EN_RXADDR,std::vector<uint8_t>(1,0));
	uint8_t en_rxaddr =data.at(1);
	en_rxaddr = en_rxaddr | (1<<2);
	nrf24_Write_Reg (spi_nrf24L01,EN_RXADDR, en_rxaddr);
	/* We must write the address for Data Pipe 1, if we want to use any pipe from 2 to 5
	 * The Address from DATA Pipe 2 to Data Pipe 5 differs only in the LSB
	 * Their 4 MSB Bytes will still be same as Data Pipe 1
	 *
	 * For Eg->
	 * Pipe 1 ADDR = 0xAABBCCDD11
	 * Pipe 2 ADDR = 0xAABBCCDD22
	 * Pipe 3 ADDR = 0xAABBCCDD33
	 *
	 */
	//nrf24_Write_Reg_multi(RX_ADDR_P1, Address, 5);  // Write the Pipe1 address
	nrf24_Write_Reg_multi(spi_nrf24L01,RX_ADDR_P1,Address);
	nrf24_Write_Reg(spi_nrf24L01,RX_ADDR_P2, 0xEE);  // Write the Pipe2 LSB address
	nrf24_Write_Reg (spi_nrf24L01,RX_PW_P2, 32);   // 32 bit payload size for pipe 2

	// power up the device in Rx mode
	nrf24_Read_Reg(spi_nrf24L01,CONFIG,std::vector<uint8_t>(1,0));
	uint8_t config =data.at(1);
	//uint8_t config = nrf24_ReadReg(CONFIG);

	config = config | (1<<1) | (1<<0);
	nrf24_Write_Reg (spi_nrf24L01,CONFIG, config);

	// Enable the chip after configuring the device
	CE_pin.SetPinLevel(LVL::HIGH);
}

uint8_t isDataAvailable (SPI& spi_nrf24L01,int pipenum)
{
	nrf24_Read_Reg(spi_nrf24L01,STATUS,std::vector<uint8_t>(1,0));
	volatile uint8_t status =data.at(1);
//	status=0x4E;
	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{
		nrf24_Write_Reg(spi_nrf24L01,STATUS, (1<<6));
		return 1;
	}
	return 0;
}


void NRF24_Receive (SPI& spi_nrf24L01,std::vector<uint8_t> data)
{
	uint8_t result = 0;
	// select the device
	// payload command
	//cmdtosend = R_RX_PAYLOAD;
	//HAL_SPI_Transmit(spi_nrf24L01, cmdtosend);
	// Receive the payload
	//HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);
	// Unselect the device
	nrf24_Read_Reg(spi_nrf24L01,R_RX_PAYLOAD,std::vector<uint8_t>(3,0));
    result=data.at(1);
	//HAL_Delay(1);    for(int i=1;i<999999;i++)
    for(int i=1;i<999999;i++)
    {
      int k=i+5-3;
    }
	//cmdtosend = FLUSH_RX;
	nrfsendCmd(spi_nrf24L01,std::vector<uint8_t>{FLUSH_RX});
}

/******************************************/
void setChannel(SPI& spi_nrf24L01,uint8_t channel)
{
 if(channel<128)
 {
  nrf24_Write_Reg(spi_nrf24L01,RF_CH,channel);
 }
}
