/**************************************************************************************************
 * @file uart.cpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.0
 * @date 31.08.2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/

#include "uart.hpp"

/**************************************************************************************************
 * @brief Настройка интерфейса
 * @return true ОК
 * @return false авария


 *************************************************************************************************/
bool UART::HardwareSettings()
{
    if (!SettingsAccident) return true;
    SettingsAccident = false;

    uint32_t APBxBusClockFreq{0};
    if (UARTx == USART1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        _vectorIRQn      = USART1_IRQn;
        APBxBusClockFreq = ClockSystem::APB2BusClock;

        if ((_TxD->_GPIOx == GPIOA) && (_RxD->_GPIOx == GPIOA))
            AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;
        else if ((_TxD->_GPIOx == GPIOB) && (_RxD->_GPIOx == GPIOB))
            AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
    }
    else if (UARTx == USART2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        _vectorIRQn      = USART2_IRQn;
        APBxBusClockFreq = ClockSystem::APB1BusClock;

        if ((_TxD->_GPIOx == GPIOA) && (_RxD->_GPIOx == GPIOA))
            AFIO->MAPR &= ~AFIO_MAPR_USART2_REMAP;
        else if ((_TxD->_GPIOx == GPIOD) && (_RxD->_GPIOx == GPIOD))
            AFIO->MAPR |= AFIO_MAPR_USART2_REMAP;
    }
    else if (UARTx == USART3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
        _vectorIRQn      = USART3_IRQn;
        APBxBusClockFreq = ClockSystem::APB1BusClock;

        if ((_TxD->_GPIOx == GPIOB) && (_RxD->_GPIOx == GPIOB))
            AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_NOREMAP;
        else if ((_TxD->_GPIOx == GPIOC) && (_RxD->_GPIOx == GPIOC))
            AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_PARTIALREMAP;
        else if ((_TxD->_GPIOx == GPIOD) && (_RxD->_GPIOx == GPIOD))
            AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_FULLREMAP;
    }
    /*else if (UARTx == UART4)
    {
        RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
        _vectorIRQn      = UART4_IRQn;
        APBxBusClockFreq = ClockSystem::APB1BusClock;
    }
    else if (UARTx == UART5)
    {
        RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
        _vectorIRQn      = UART5_IRQn;
        APBxBusClockFreq = ClockSystem::APB1BusClock;
    }*/

    SettingsAccident += !_TxD->Settings(TYPE::OUTPUT_AFO_Push_Pull);
    SettingsAccident += !_RxD->Settings(TYPE::INPUT_Pull_Down, LVL::HIGH);

    if (SettingsAccident) return false;

    UARTx->BRR = GetBRRsettings(APBxBusClockFreq, BaudRate);

    UARTx->CR3 = 0;
    UARTx->CR2 = 0;

    UARTx->CR1 = 0;
    UARTx->CR1 |= USART_CR1_TE;
    UARTx->CR1 |= USART_CR1_RE;
    UARTx->CR1 |= USART_CR1_UE;

    return true;
}

bool UART::AdditionalSettings(AdditionalSetting dma_tx,
                              AdditionalSetting dma_rx,
                              AdditionalSetting idle_irq,
                              AdditionalSetting rxne_irq,
                              AdditionalSetting txe_irq)
{
    NVIC_DisableIRQ(_vectorIRQn);

    UARTx->CR1 &= ~USART_CR1_UE;
    UARTx->CR1 &= ~USART_CR1_RE;
    UARTx->CR1 &= ~USART_CR1_TE;
    UARTx->CR3 = 0;
    UARTx->CR3 |= static_cast<uint32_t>(dma_tx);
    UARTx->CR3 |= static_cast<uint32_t>(dma_rx);

    UARTx->CR1 |= USART_CR1_TE;
    UARTx->CR1 |= USART_CR1_RE;
    UARTx->CR1 |= USART_CR1_UE;
    UARTx->CR1 |= static_cast<uint32_t>(idle_irq);
    UARTx->CR1 |= static_cast<uint32_t>(rxne_irq);
    UARTx->CR1 |= static_cast<uint32_t>(txe_irq);

    NVIC_EnableIRQ(_vectorIRQn);
    return true;
}


/**************************************************************************************************
 * @brief
 *
 * @param packet
 * @return std::vector<uint8_t>
 *************************************************************************************************/
std::vector<uint8_t> UART::Transmitt(std::vector<uint8_t> &packet)
{
    ProgrammTimer timeout(2);
    if (!HardwareSettings()) return {};

    if (UARTx->CR1 & USART_CR1_RXNEIE)
    {
        NVIC_EnableIRQ(USART1_IRQn);
        UART::Reciever(0, UART::BufferMode::RESET);
    }

    while (!(UARTx->SR & USART_SR_TXE) && !timeout.TimeOutEvent()) {}
    for (uint8_t &byte : packet)
    {
        UARTx->DR = byte;
        while (!(UARTx->SR & USART_SR_TXE) && !timeout.TimeOutEvent()) {}
    }

    return {};
}
/**************************************************************************************************
 * @brief
 *
 * @param packet приемный буфер (необходимо push-backнуть размер принимаемой посылки)
 * @return std::vector<uint8_t>
 *************************************************************************************************/
std::vector<uint8_t> UART::Recieve(std::vector<uint8_t> &packet)
{
    if (!HardwareSettings()) return {};

    ProgrammTimer timeout(2);
    uint8_t size = packet.back();
    packet.clear();
    bool complete{false};

    while (!complete && !timeout.TimeOutEvent())
    {
        if (UARTx->CR1 & USART_CR1_RXNEIE)
        {
            packet   = UART::Reciever(0, UART::BufferMode::RELOAD);
            complete = (packet.size() < size) ? (false) : (true);
        }
        else
        {
            while (!(UARTx->SR & USART_SR_RXNE) && !timeout.TimeOutEvent()) {}
            packet.push_back(UARTx->DR);
            complete = (packet.size() < size) ? (false) : (true);
        }
    }
    if (UARTx->CR1 & USART_CR1_RXNEIE)
    {
        NVIC_DisableIRQ(USART1_IRQn);
        UART::Reciever(0, UART::BufferMode::RESET);
    }
    // if (timeout.TimeOutEvent()) MESSAGE_TRACE();

    if (!complete) packet.clear();
    return packet;
}

/**************************************************************************************************
 * @brief
 *
 * @param byte
 * @param mode
 * @return std::vector<uint8_t>
 *************************************************************************************************/
std::vector<uint8_t> UART::Reciever(uint8_t byte, BufferMode mode)
{
    static std::vector<uint8_t> buffer{};
    switch (mode)
    {
        case UART::BufferMode::RESET: buffer.clear(); break;
        case UART::BufferMode::RELOAD: return buffer; break;
        case UART::BufferMode::PUSH: buffer.push_back(byte); break;
        default: break;
    }
    return buffer;
}

// extern "C" void UART1_IRQHandler(void)
//{
//     if (!(MDR_UART1->RIS & UART_RIS_RXRIS)) return;
//     UART::Reciever(MDR_UART1->DR, UART::BufferMode::PUSH);
//     return;
// }

/**********************************************************************************************
 * @brief Функция расчета предделителя USART
 * Выполняет расчет предделителя (регистр USARTx->BRR) исходя из частоты
 * тактирования шины APB2 и необходимой скорости обмена (бод/с)
 * @param ClockFreq тактовая частота шины APB2
 * @param baud_rate скорость обмена (бод/с)
 * @return uint16_t значение предделителя регистра BRR
 **********************************************************************************************/
uint16_t UART::GetBRRsettings(uint32_t ClockFreq, double baud_rate)
{
    double UsartDiv = static_cast<double>(ClockFreq) / (baud_rate);
    return roundf(UsartDiv);
}

bool UART::CheckFlagIDLE()
{
    bool f{UARTx->SR & USART_SR_IDLE};
    UARTx->SR &= ~USART_SR_IDLE;
    return f;
}
bool UART::CheckFlagRXNE()
{
    bool f{UARTx->SR & USART_SR_RXNE};
    UARTx->SR &= ~USART_SR_RXNE;
    return f;
}
bool UART::CheckFlagTXE()
{
    bool f{UARTx->SR & USART_SR_TXE};
    UARTx->SR &= ~USART_SR_TXE;
    return f;
}
bool UART::ReadSRandDR()
{
    (void)UARTx->SR;
    (void)UARTx->DR;
    return true;
}

UART::~UART()
{
    UARTx->BRR  = 0;
    UARTx->CR1  = 0;
    UARTx->CR2  = 0;
    UARTx->CR3  = 0;
    UARTx->GTPR = 0;
    UARTx->SR   = 0;

    return;
}
