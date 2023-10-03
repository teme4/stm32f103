#include <stm32f1xx.h>
#include "hardware_config.h"
#include "gpio.h"


uint8_t spi_transmit(uint16_t data)
{
while (!(SPI1->SR & SPI_SR_TXE));
SPI1->DR = data;
  // Ждём получения данных, читаем их.
while(!(SPI1->SR&SPI_SR_RXNE)) {}
data=SPI1->DR;
return data;
}

void CS_Select (void)
{	//HAL_GPIO_WritePin( GPIO_PIN_RESET);
	Set_pin_L(GPIOA, SPI1_NSS);
}

void CS_UnSelect (void)
{	//HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
	Set_pin_H(GPIOA, SPI1_NSS);
}

void CE_Enable (void)
{//HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
	Set_pin_H(GPIOA, SPI1_CE);	
}

void CE_Disable (void)
{//HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
	Set_pin_L(GPIOA, SPI1_CE);	
}

void nrf24_WriteReg (uint8_t Reg, uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	buf[1] = Data;

	// Pull the CS Pin LOW to select the device
	CS_Select();
	//HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);
    spi_transmit(buf[0]);
    spi_transmit(buf[1]);
	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

uint8_t nrf24_ReadReg (uint8_t Reg)
{
	uint8_t data=0;

	// Pull the CS Pin LOW to select the device
	CS_Select();
	data=spi_transmit(Reg);
	// Pull the CS HIGH to release the device
	CS_UnSelect();
	return data;
}

void spi_init()
{
 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; //--- Включаем тактирование альтернативных функции
 RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //--- Включаем тактированние SPI1 
// Указываем полярность и фазу тактового сигнала SPI.
const uint32_t CPOL=SPI_CR1_CPOL*0;
const uint32_t CPHA=SPI_CR1_CPHA*0;

// Конфигурируем SPI1 (обычный ведущий режим в данном случае).
    // BIDIMODE: 0 (выбор режима с одной линией данных - отключено);
    // BIDIOE: 0 (направление передачи, бит используется при BIDIMODE=1);
    // CRCEN: 0 (аппаратный подсчёт CRC отключён);
    // CRCNEXT:0 (отправка CRC, используется при CRCEN=1;
    // DFF: 0 (длина фрейма данных, здесь - 8-битовый фрейм);
    // RXONLY: 0 (включение режима "только приём", здесь - полнодуплекс);
    // SSM: 0 (включение режима программного управления сигналом NSS);
    // SSI: 0 (значение бита используется вместо сигнала NSS при SSM=1);
    // LSBFIRST: 0 (порядок передачи битов, здесь - первым передаётся старший);
    // SPE: 0 (бит включения SPI, здесь разделяем конфигурирование и включение);
    // BR[2:0] (управление скоростью передачи, только для ведущего;
            // здесь задаём 0x7, что соотв. макс. делителю /256 и мин. скорости);
    // MSTR: 1 (бит переключения в ведущий режим).
    SPI1->CR1=              // Большинство битов задаём нулевыми.
            SPI_CR1_MSTR|   // Ведущий режим.
            SPI_CR1_BR|     // BR[2:0]=0x7 - минимальная скорость для теста.
            CPOL|           // Полярность тактового сигнала.
            CPHA;           // Фаза тактового сигнала.

    // С помощью регистра CR2 настраиваем генерацию запросов на
    // прерывание и DMA (если нужно); с помощью бита SSOE запрещаем или
    // разрешаем использовать ведущему устройству вывод NSS как выход.
    SPI1->CR2&=~0xE7;       // Сбрасываем все значимые биты регистра.
    SPI1->CR2|=SPI_CR2_SSOE;// NSS будет выходом.

    // Включаем SPI1.
    // Передача не начнётся, пока не запишем что-то в его регистр,
    // данных, но установится состояние выходов.
    SPI1->CR1|=SPI_CR1_SPE;

}


int main(void) 
{
 //gpio_init();
 //spi_init();
 //nrf24_ReadReg(CONFIG);
uint16_t k=0;
k++;


 while(1) 
 {

 }
 }