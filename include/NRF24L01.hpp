/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  NRF24L01.h
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

//NRF pins

//1 - +3.3V
//2 - GND
//3 - CE      A4
//4 - CS      A3
//5 - SCK     A5
//6 - MOSI    A7
//7 - MISO    A6
//8 - IRQ

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include <stm32f1xx.h>
#include <array>
#include <vector>
#include <DigitalInterface/drivers.hpp>
#include <Time&Sync/drivers.hpp>

//void nrf24_WriteReg (uint8_t Reg, uint8_t Data);
//void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size);
//void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size);
void nrfsendCmd (SPI& spi_nrf24L01,std::vector<uint8_t> Buffer_tx);
void NRF24_Init (SPI& spi_nrf24L01);

void NRF24_TxMode (SPI& spi_nrf24L01,std::vector<uint8_t> Address, uint8_t channel);
void NRF24_RxMode (SPI& spi_nrf24L01, std::vector<uint8_t> Address, uint8_t channel);

uint8_t isDataAvailable (SPI& spi_nrf24L01,int pipenum);
void nrf24_Write_Reg(SPI& spi_nrf24L01,uint8_t reg,uint8_t value);
void nrf24_Write_Reg_multi(SPI& spi_nrf24L01,uint8_t reg,std::vector<uint8_t> Buffer_tx);
void nrf24_Read_Reg(SPI& spi_nrf24L01,uint8_t reg,std::vector<uint8_t> Buffer_rx);
uint8_t nrf24_reset(SPI& spi_nrf24L01,uint8_t REG);
void NRF24_Receive (SPI& spi_nrf24L01,std::vector<uint8_t> data);
uint8_t NRF24_Transmit (SPI& spi_nrf24L01,std::vector<uint8_t> data);


void NRF24_RxMode2 (SPI& spi_nrf24L01, std::vector<uint8_t> Address, uint8_t channel);
void NRF24_TxMode2 (SPI& spi_nrf24L01,std::vector<uint8_t> Address, uint8_t channel);



/******************************************/
void setChannel(SPI& spi_nrf24L01,uint8_t channel);

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	      0x1C
#define FEATURE	    0x1D

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF



       // #define MASK_RX_DR  6 //вкл/откл прерывание от бита RX_DR в рег. STATUS. 0-вкл, 1-выкл.
       // #define MASK_TX_DS  5 //вкл/откл прерывание от бита TX_DS в рег. STATUS. 0-вкл, 1-выкл.
        //#define MASK_MAX_RT 4 //вкл/откл прерывание от бита MAX_RT в рег. STATUS. 0-вкл, 1-выкл.
        #define EN_CRC      3 //включение CRC. По умолчанию вкл. если один из битов регистра EN_AA включен.
        #define CRCO        2 //режим CRC. 0-1 байт, 1-2 байта.
        #define PWR_UP      1 //1-POWER UP, 0-POWER DOWN, по умолчанию 0.
        #define PRIM_RX     0 //0-режим передачи, 1-режим приема.

#endif /* INC_NRF24L01_H_ */