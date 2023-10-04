#include "stm32f1xx.h"
#include "hardware_config.h"
#include "gpio.h"


void setupPin(GPIO_TypeDef * port, uint32_t pin, uint32_t mode){
 RCC->APB2ENR |= (1 << (((uint32_t)port - APB2PERIPH_BASE) / 0x400));
 if (pin > 7){
  port->CRH &= ~(0xf  << ((pin - 8) * 4));
  port->CRH |= mode << ((pin - 8) * 4);
 }
 else{
  port->CRL &= ~(0xf << (pin * 4));
  port->CRL |= mode << (pin * 4);
 }
}

void Set_pin_H(GPIO_TypeDef *port,uint8_t pin)
{
  port->ODR |= (1<<pin);
}

void Set_pin_L(GPIO_TypeDef *port,uint8_t pin)
{
  port->ODR&= ~ (1<<pin);
}

int get_state_Pin (GPIO_TypeDef *port, uint8_t pin)
    {

	uint16_t mask;
	mask = ( 1<< pin);
	if ((port->IDR) & mask) return 1;
	else return 0;
}



void gpio_init()
{
    // PA4, SPI1_NSS: alt. out, push-pull, high speed
    // PA5, SPI1_SCK: alt. out, push-pull, high speed
    // PA6, SPI1_MISO: input, pull up/down
    // PA7, SPI1_MOSI: alt. out, push-pull, high speed
setupPin(GPIOA,SPI1_NSS,gpio_mode_pp_50);//SPI1_NSS
setupPin(GPIOA,SPI1_CE,gpio_mode_pp_50);//SPI_CE
setupPin(GPIOA,SPI1_SCK,alternate_mode_pp_50);//SPI1_SCK
setupPin(GPIOA,SPI1_MISO,input_mode_pull_down_up);
setupPin(GPIOA,SPI1_MOSI,alternate_mode_pp_50);
}