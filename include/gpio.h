#ifndef _GPIO_H_
#define _GPIO_H_

#include "stm32f1xx.h"
#include "hardware_config.h"

#define SPI1_CE    3
#define SPI1_NSS   4
#define SPI1_SCK   5
#define SPI1_MISO  6
#define SPI1_MOSI  7


void gpio_init(void);
void Set_pin_L(GPIO_TypeDef *port,uint8_t pin);
void Set_pin_H(GPIO_TypeDef *port,uint8_t pin);
void uart_tx_data(unsigned char * data);
int get_state_Pin (GPIO_TypeDef *port, uint8_t pin);

#endif //_GPIO_H_