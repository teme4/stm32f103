#include "stm32f1xx.h"
#include "hardware_config.h"

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
  /*
setupPin(lcd_port,lcd_d0,lcd_mode);
setupPin(lcd_port,lcd_d1,lcd_mode);
setupPin(lcd_port,lcd_d2,lcd_mode);
setupPin(lcd_port,lcd_d3,lcd_mode);
setupPin(lcd_port,lcd_d4,lcd_mode);
setupPin(lcd_port,lcd_d5,lcd_mode);
setupPin(lcd_port,lcd_d6,lcd_mode);
setupPin(lcd_port,lcd_d7,lcd_mode);

setupPin(lcd_control_port,lcd_RST,lcd_mode);
setupPin(lcd_control_port,lcd_CS,lcd_mode);
setupPin(lcd_control_port,lcd_RS,lcd_mode);
setupPin(lcd_control_port,lcd_WR,lcd_mode);
setupPin(lcd_control_port,lcd_RD,lcd_mode);
//Set_pin_H(lcd_control_port,lcd_RD);
*/
}