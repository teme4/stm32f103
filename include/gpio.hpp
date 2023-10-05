#ifndef _GPIO_H_
#define _GPIO_H_

#include "stm32f1xx.h"
#include "hardware_config.h"

//            General purpose output
#define gpio_mode_pp_2						0x1
#define gpio_mode_pp_10						0x2
#define gpio_mode_pp_50						0x3

#define gpio_mode_od_2						0x5
#define gpio_mode_od_10						0x6
#define gpio_mode_od_50						0x7

//             Alternate Function output
#define alternate_mode_pp_2					0x9
#define alternate_mode_pp_10				0xA
#define alternate_mode_pp_50				0xB

#define alternate_mode_od_2					0xD
#define alternate_mode_od_10				0xE
#define alternate_mode_od_50				0xF

//                Input
#define input_mode_analog				    0
#define input_mode_floating				    0x4
#define input_mode_pull_down_up			    0x8

#define SPI1_CE    4
#define SPI1_NSS   3
#define SPI1_SCK   5
#define SPI1_MISO  6
#define SPI1_MOSI  7


void gpio_init(void);
void Set_pin_L(GPIO_TypeDef *port,uint8_t pin);
void Set_pin_H(GPIO_TypeDef *port,uint8_t pin);
void uart_tx_data(unsigned char * data);
int get_state_Pin (GPIO_TypeDef *port, uint8_t pin);

#endif //_GPIO_H_