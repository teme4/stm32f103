/**************************************************************************************************
 * @file i2c.cpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.0
 * @date 22.12.2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/
#include "i2c.hpp"

/**************************************************************************************************
 * @brief
 *
 * @param packet
 * @return std::vector<uint8_t> {} - complete; {*,**} - error
 *************************************************************************************************/
std::vector<uint8_t> I2C::Transmitt(std::vector<uint8_t> &packet)
{
    if (_I2Cx->SR2 & I2C_SR2_BUSY) _HardwareSettings();
    ProgrammTimer timeout(_TimeOut);

    _I2Cx->CR1 &= ~I2C_CR1_POS;
    _I2Cx->CR1 |= I2C_CR1_ACK;
    _I2Cx->CR1 |= I2C_CR1_START;
    while (!(_I2Cx->SR1 & I2C_SR1_SB) && !timeout.TimeOutEvent()) {}
    (void)_I2Cx->SR1;///< ENV_5

    _I2Cx->DR = (packet.front());
    while (!(_I2Cx->SR1 & I2C_SR1_ADDR) && !timeout.TimeOutEvent())
    {
        if (_I2Cx->SR1 & I2C_SR1_AF) return packet;
    }
    (void)_I2Cx->SR1;
    (void)_I2Cx->SR2;
    packet.erase(packet.begin());///< ENV_6

    for (uint8_t &byte : packet)
    {
        while (!(_I2Cx->SR1 & I2C_SR1_TXE) && !timeout.TimeOutEvent()) {}///< ENV_8
        _I2Cx->DR = byte;
    }

    while (!(_I2Cx->SR1 & I2C_SR1_BTF) && !timeout.TimeOutEvent()) {}///< ENV_8_2
    _I2Cx->CR1 |= I2C_CR1_STOP;
    while (!(_I2Cx->CR1 & I2C_CR1_STOP) && !timeout.TimeOutEvent()) {}
    (void)_I2Cx->SR1;

    if (timeout.TimeOutEvent()) return packet;
    packet.clear();

    return packet;
}

/**************************************************************************************************
 * @brief
 *
 * @param packet {addr, reg, size}
 * @return std::vector<uint8_t> {*,**} - complete; {} - error
 *************************************************************************************************/
std::vector<uint8_t> I2C::Recieve(std::vector<uint8_t> &packet)
{
    if (_I2Cx->SR2 & I2C_SR2_BUSY) _HardwareSettings();
    uint8_t size{packet.back()};
    packet.pop_back();
    uint8_t reg_addr{packet.back()};
    packet.pop_back();
    uint8_t addr{packet.back()};
    packet.clear();
    packet.reserve(size);

    ProgrammTimer timeout(_TimeOut);

    _I2Cx->CR1 &= ~I2C_CR1_POS;
    _I2Cx->CR1 |= I2C_CR1_ACK;
    _I2Cx->CR1 |= I2C_CR1_START;
    while (!(_I2Cx->SR1 & I2C_SR1_SB) && !timeout.TimeOutEvent()) {}
    (void)_I2Cx->SR1;///< ENV_5

    _I2Cx->DR = addr;
    while (!(_I2Cx->SR1 & I2C_SR1_ADDR) && !timeout.TimeOutEvent())
        if (_I2Cx->SR1 & I2C_SR1_AF) return packet;

    (void)_I2Cx->SR1;
    (void)_I2Cx->SR2;

    _I2Cx->DR = reg_addr;
    while (!(_I2Cx->SR1 & I2C_SR1_TXE) && !timeout.TimeOutEvent()) {};
    /**************************************************************************************************
     * @brief Повторный старт
     ***************************************************************************************************/
    _I2Cx->CR1 |= I2C_CR1_START;
    while (!(_I2Cx->SR1 & I2C_SR1_SB) && !timeout.TimeOutEvent()) {}
    (void)_I2Cx->SR1;///< ENV_5

    _I2Cx->DR = (addr + 1);
    while (!(_I2Cx->SR1 & I2C_SR1_ADDR) && !timeout.TimeOutEvent())
    {
        if (_I2Cx->SR1 & I2C_SR1_AF) return packet;
    }
    (void)_I2Cx->SR1;
    (void)_I2Cx->SR2;///< ENV_6_3

    size--;
    while (packet.size() < size)
    {
        while (!(_I2Cx->SR1 & I2C_SR1_RXNE) && !timeout.TimeOutEvent()) {}
        packet.push_back(_I2Cx->DR);
    }

    _I2Cx->CR1 &= ~I2C_CR1_ACK;
    _I2Cx->CR1 |= I2C_CR1_STOP;
    while (!(_I2Cx->SR1 & I2C_SR1_RXNE) && !timeout.TimeOutEvent()) {}
    packet.push_back(_I2Cx->DR);

    if (timeout.TimeOutEvent()) packet.clear();

    return packet;
}

void I2C::_TrubleHandler()
{
    _I2Cx->CR1 |= I2C_CR1_PE;
    _I2Cx->CR1 &= ~I2C_CR1_PE;

    _pinSCL->Settings(TYPE::OUTPUT_GPO_Open_Drain, LVL::HIGH);
    _pinSDA->Settings(TYPE::OUTPUT_GPO_Open_Drain, LVL::HIGH);
    Timers::delay(500, Timers::MICROSECOND);

    _pinSCL->SetPinLevel(LVL::LOW);
    _pinSDA->SetPinLevel(LVL::LOW);
    Timers::delay(500, Timers::MICROSECOND);

    _pinSCL->SetPinLevel(LVL::HIGH);
    _pinSDA->SetPinLevel(LVL::HIGH);
    Timers::delay(500, Timers::MICROSECOND);

    _pinSCL->Settings(TYPE::OUTPUT_AFO_Open_Drain, LVL::HIGH);
    _pinSDA->Settings(TYPE::OUTPUT_AFO_Open_Drain, LVL::HIGH);
}

bool I2C::_HardwareSettings()
{
    const double nano        = static_cast<double>(1) / 1000000000;
    const uint32_t FPCLK1    = ClockSystem::APB1BusClock;
    const double T_pinSCL    = static_cast<double>(1) / Frequency / nano;
    const double TPCLK1      = static_cast<double>(1) / FPCLK1 / nano;
    const uint16_t I2C_Clock = static_cast<uint16_t>(FPCLK1 / 1000000);

    const uint16_t Prescaler = static_cast<uint16_t>(T_pinSCL / (2 * TPCLK1));
    const uint16_t TRise = static_cast<uint16_t>(static_cast<double>((Frequency == 100000) ? 1000 : 300) / TPCLK1) + 1;

    if (_I2Cx == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    if (_I2Cx == I2C2) RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    _TrubleHandler();

    _I2Cx->CR1 &= ~I2C_CR1_PE;
    _I2Cx->CR1 |= I2C_CR1_SWRST;
    _I2Cx->CR1 &= ~I2C_CR1_SWRST;
    _I2Cx->CR2   = I2C_Clock;
    _I2Cx->CCR   = Prescaler;
    _I2Cx->TRISE = TRise;
    _I2Cx->CR1 |= I2C_CR1_SMBTYPE;
    _I2Cx->CR1 |= I2C_CR1_PE;
    _I2Cx->CR1 |= I2C_CR1_ACK;

    return true;
}
