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

void nrf24_Read_Reg(SPI& spi_nrf24L01,uint8_t reg,std::vector<uint8_t> Buffer_rx)
{
 Buffer_rx.reserve(1);
 Buffer_rx.insert(Buffer_rx.begin(),reg|R_REGISTER);
volatile uint8_t size2=Buffer_rx.size();
 spi_nrf24L01.Recieve(Buffer_rx);
 ptr= reinterpret_cast<uint8_t*>(data.data());
 size2=Buffer_rx.size();
 for (uint8_t i=0;i<size2;i++)
 {
   data.at(i)=Buffer_rx.at(i);
 }
 //Buffer_rx.clear();
}
//******************************************************************//
void nrf24_Write_Reg(SPI& spi_nrf24L01,uint8_t reg,uint8_t value)
{
 static std::vector<uint8_t> Buffer_tx;
 Buffer_tx.clear();
 Buffer_tx.resize(2);
 Buffer_tx.reserve(1);
 Buffer_tx.insert(Buffer_tx.begin(),reg|W_REGISTER);
 Buffer_tx.insert(Buffer_tx.begin()+1,value);
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

uint8_t spi_transmit(uint8_t data)
{

}

void nrf24_Write_Reg (uint8_t Reg, uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = Reg | W_REGISTER;
	buf[1] = Data;
  spi_transmit(buf[0]);
  spi_transmit(buf[1]);
}

uint8_t nrf24_ReadReg (uint8_t Reg)
{
  uint8_t data=0;
  data=spi_transmit(Reg | R_REGISTER);
  if (data!=STATUS)//если адрес равен адрес регистра статус то и возварщаем его состояние
  {
    data=spi_transmit(0xFF);
  }
  return data;
}



/* Read multiple bytes from the register */
/*
void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size)
{
	volatile uint8_t buf[32]={0,};
    data=spi_transmit(Reg | R_REGISTER);
  if (data!=STATUS)//если адрес равен адрес регистра статус то и возварщаем его состояние
  {
	for(int i=0;i<size;i++)
	{
	buf[i]= spi_transmit(0xFF);
	}
  }
}*/



// send the command to the NRF
void nrfsendCmd (uint8_t cmd)
{
	spi_transmit(cmd);
}


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

	else {
	nrf24_Write_Reg(spi_nrf24L01,CONFIG, 0x08);
	check=nrf24_ReadReg(CONFIG);
	if(check!=0x08)
	{
		return 0;
	}
	
	nrf24_Write_Reg(spi_nrf24L01,EN_AA, 0x3F);
	nrf24_Write_Reg(spi_nrf24L01,EN_RXADDR, 0x03);
	nrf24_Write_Reg(spi_nrf24L01,SETUP_AW, 0x03);
	nrf24_Write_Reg(spi_nrf24L01,SETUP_RETR, 0x03);
	nrf24_Write_Reg(spi_nrf24L01,RF_CH, 0x02);
	nrf24_Write_Reg(spi_nrf24L01,RF_SETUP, 0x0E);
	nrf24_Write_Reg(spi_nrf24L01,STATUS, 0x00);
	nrf24_Write_Reg(spi_nrf24L01,OBSERVE_TX, 0x00);
	nrf24_Write_Reg(spi_nrf24L01,CD, 0x00);
	//uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	//nrf24_Write_RegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
	//uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	//nrf24_Write_RegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_Write_Reg(spi_nrf24L01,RX_ADDR_P2, 0xC3);
	nrf24_Write_Reg(spi_nrf24L01,RX_ADDR_P3, 0xC4);
	nrf24_Write_Reg(spi_nrf24L01,RX_ADDR_P4, 0xC5);
	nrf24_Write_Reg(spi_nrf24L01,RX_ADDR_P5, 0xC6);
	//uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	//nrf24_Write_RegMulti(TX_ADDR, tx_addr_def, 5);
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
	//CE_Disable();
	nrf24_reset (spi_nrf24L01,0);
	nrf24_Write_Reg(spi_nrf24L01,CONFIG, 0);  // will be configured later
	nrf24_Write_Reg(spi_nrf24L01,EN_AA, 0);  // No Auto ACK
	nrf24_Write_Reg (spi_nrf24L01,EN_RXADDR, 0);  // Not Enabling any data pipe right now
	nrf24_Write_Reg (spi_nrf24L01,SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address
	nrf24_Write_Reg (spi_nrf24L01,SETUP_RETR, 0);   // No retransmission
	nrf24_Write_Reg (spi_nrf24L01,RF_CH, 0);  // will be setup during Tx or RX
	nrf24_Write_Reg (spi_nrf24L01,RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps
	//CE_Enable();
}


// set up the Tx mode

void NRF24_TxMode (uint8_t *Address, uint8_t channel)
{
	//CE_Disable();
	nrf24_Write_Reg (RF_CH, channel);  // select the channel
	//nrf24_Write_Reg_multi(TX_ADDR, Address, 5);  // Write the TX address
	// power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
//	config = config | (1<<1);   // write 1 in the PWR_UP bit
	config = config & (0xF2);    // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
	nrf24_Write_Reg (CONFIG, config);
	//CE_Enable();
}


// transmit the data
/*
uint8_t NRF24_Transmit (uint8_t *data)
{
	uint8_t cmdtosend = 0;

	// select the device
	

	// payload command
	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	// send the payload
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

	// Unselect the device
	

	HAL_Delay(1);

	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
	if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
	{
		cmdtosend = FLUSH_TX;
		nrfsendCmd(cmdtosend);

		// reset FIFO_STATUS
		nrf24_reset (FIFO_STATUS);

		return 1;
	}

	return 0;
}*/


void NRF24_RxMode (uint8_t *Address, uint8_t channel)
{
	// disable the chip before configuring the device
	//CE_Disable();

	//nrf24_reset (STATUS);

	nrf24_Write_Reg (RF_CH, channel);  // select the channel

	// select data pipe 2
	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<2);
	nrf24_Write_Reg (EN_RXADDR, en_rxaddr);

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
	nrf24_Write_Reg(RX_ADDR_P2, 0xEE);  // Write the Pipe2 LSB address

	nrf24_Write_Reg (RX_PW_P2, 32);   // 32 bit payload size for pipe 2


	// power up the device in Rx mode
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24_Write_Reg (CONFIG, config);

	// Enable the chip after configuring the device
	//CE_Enable();
}

uint8_t isDataAvailable (int pipenum)
{
	uint8_t status = nrf24_ReadReg(STATUS);
	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{
		nrf24_Write_Reg(STATUS, (1<<6));
		return 1;
	}
	return 0;
}

/*
void NRF24_Receive (uint8_t *data)
{
	uint8_t cmdtosend = 0;
	// select the device
	
	// payload command
	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);
	// Receive the payload
	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);
	// Unselect the device
	
	HAL_Delay(1);
	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);
}*/



// Read all the Register data
/*
void NRF24_ReadAll (uint8_t *data)
{
	for (int i=0; i<10; i++)
	{
		*(data+i) = nrf24_ReadReg(i);
	}

	nrf24_ReadReg_Multi(RX_ADDR_P0, (data+10), 5);

	nrf24_ReadReg_Multi(RX_ADDR_P1, (data+15), 5);

	*(data+20) = nrf24_ReadReg(RX_ADDR_P2);
	*(data+21) = nrf24_ReadReg(RX_ADDR_P3);
	*(data+22) = nrf24_ReadReg(RX_ADDR_P4);
	*(data+23) = nrf24_ReadReg(RX_ADDR_P5);

	nrf24_ReadReg_Multi(RX_ADDR_P0, (data+24), 5);

	for (int i=29; i<38; i++)
	{
		*(data+i) = nrf24_ReadReg(i-12);
	}

}*/