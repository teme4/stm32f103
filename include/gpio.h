#ifndef _GPIO_H_
#define _GPIO_H_

#include "stm32f1xx.h"
#include "hardware_config.h"

void gpio_init(void);
void Set_pin_L(GPIO_TypeDef *port,uint8_t pin);
void Set_pin_H(GPIO_TypeDef *port,uint8_t pin);
void uart_tx_data(unsigned char * data);
int get_state_Pin (GPIO_TypeDef *port, uint8_t pin);

#endif //_GPIO_H_