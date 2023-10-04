/**************************************************************************************************
 * @file spi->cpp
 * @author Yakovlev Vladislav (y->yakovlev@quanttelecom->ru)
 * @brief
 * @version X->x
 * @date 11->07->2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/
#include "spi.hpp"


std::vector<uint8_t> SPI::Transmitt(std::vector<uint8_t> &packet) { return Recieve(packet); }

/**************************************************************************************************
 * @brief Запрос данных
 * @param packet - вектор, содержащий
 * @return std::vector<uint8_t>
 *************************************************************************************************/
std::vector<uint8_t> SPI::Recieve(std::vector<uint8_t> &packet)
{
    if (!CheckSettings()) return {};
    ProgrammTimer timeout(0.2);
    ChipSelect(LVL::LOW);
    for (uint8_t &byte : packet)
    {
        while (!(SPIx->SR & SPI_SR_TXE) && !timeout.TimeOutEvent()) {}
        SPIx->DR = byte;
        while (!(SPIx->SR & SPI_SR_RXNE) && !timeout.TimeOutEvent()) {}
        byte = SPIx->DR;
    }
    while ((SPIx->SR & SPI_SR_BSY) && !timeout.TimeOutEvent()) {}
    ChipSelect(LVL::HIGH);

    // if (timeout.TimeOutEvent()) MESSAGE_TRACE();

    return packet;
}

/**************************************************************************************************
 * @brief
 *
 * @return true operation was successful
 * @return false operation was failed
 *************************************************************************************************/
bool SPI::CheckSettings()
{
    if (!HardwareSettings()) return false;

    if (((SPIx->CR1 == RegisterCR1) && (SPIx->CR2 == RegisterCR2) && (SPIx->RXCRCR == RegisterRXCRCR) &&
         (SPIx->TXCRCR == RegisterTXCRCR) && (SPIx->CRCPR == RegisterCPSR)))
        return true;

    SPIx->CR1    = 0;
    SPIx->CR1    = RegisterCR1;
    SPIx->CR2    = RegisterCR2;
    SPIx->CRCPR  = RegisterCPSR;
    SPIx->RXCRCR = RegisterRXCRCR;
    SPIx->TXCRCR = RegisterTXCRCR;
    SPIxEnable(RegCR1::ACTIVE);

    return true;
}


/**************************************************************************************************
 * @brief Настройка физического уровня
 *
 * @return true успешно
 * @return false ошибка инициализации линии SPI
 *************************************************************************************************/
bool SPI::HardwareSettings()
{
    if (!SettingsAccident) return true;
    SettingsAccident = false;

    SettingsAccident += !_pinMOSI->Settings(TYPE::OUTPUT_AFO_Push_Pull);
    SettingsAccident += !_pinMISO->Settings(TYPE::INPUT_Floating);
    SettingsAccident += !_pinSCLK->Settings(TYPE::OUTPUT_AFO_Push_Pull);
    SettingsAccident += !_pinNSS->Settings(TYPE::OUTPUT_GPO_Push_Pull, LVL::HIGH);

    if (SettingsAccident) MESSAGE_FATAL_ERROR("SPI::HardwareSettings() return %d", SettingsAccident);

    return !SettingsAccident;
}

/**************************************************************************************************
 * @brief Initialisation SPI::SPI object
 *
 * @param SSE Разрешение работы приемопередатчика
 * @param MS Выбор ведущего или ведомого режима работы
 * @param frequency speed Mbps
 * @param Type Формат информационного кадра (+режим SPI)
 * @param WordSize Размер слова данных
 * @param LsbMsbFirst Направление сдвига
 * @param irq Список событий на прерывание
 * @param TxDmacr Использование DMA передатчика
 * @param RxDmacr Использование DMA приемника
 *************************************************************************************************/
void SPI::SettingsSPI(RegCR1 SPE,
                      RegCR1 MS,
                      double frequency,
                      RegCR1 Type,
                      RegCR1 WordSize,
                      RegCR1 LsbMsbFirst,
                      std::vector<IRQnSPI> irq,
                      RegDMACR TxDmacr,
                      RegDMACR RxDmacr)
{
    if (!HardwareSettings()) return;
    if (!ChipSelect(LVL::HIGH)) return;
    Frequency = frequency;

    SPIx->CR1    = 0;
    SPIx->CR2    = 0;
    SPIx->SR     = 0;
    SPIx->CRCPR  = 0;
    SPIx->RXCRCR = 0;
    SPIx->TXCRCR = 0;
    BRR          = static_cast<uint8_t>(log2(F_SPICLK / (Frequency * 1000000)) - 1);

    SPIx->CR1 |= (BRR << SPI_CR1_BR_Pos);
    SPIx->CR1 |= static_cast<uint32_t>(Type);
    SPIx->CR1 |= static_cast<uint32_t>(WordSize);
    SPIx->CR1 |= static_cast<uint32_t>(LsbMsbFirst);
    SPIx->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;
    SPIx->CR1 |= static_cast<uint32_t>(MS);

    SPIx->CR2 |= static_cast<uint32_t>(TxDmacr);
    SPIx->CR2 |= static_cast<uint32_t>(RxDmacr);

    SPIx->CR1 |= static_cast<uint32_t>(SPE);

    RegisterCR1    = SPIx->CR1;
    RegisterCR2    = SPIx->CR2;
    RegisterCPSR   = SPIx->CRCPR;
    RegisterRXCRCR = SPIx->RXCRCR;
    RegisterTXCRCR = SPIx->TXCRCR;

    return;
}


void SPI::SPIxEnable(RegCR1 sse)
{
    if (sse == RegCR1::ACTIVE) SPIx->CR1 |= static_cast<uint32_t>(RegCR1::ACTIVE);
    if (sse == RegCR1::INACTIVE) SPIx->CR1 &= ~static_cast<uint32_t>(RegCR1::ACTIVE);

    RegisterCR1 = SPIx->CR1;
    return;
}


void SPI::SetRegisterCR1(RegCR1 type, RegCR1 wordsize)
{
    if (SPIx->CR1 & static_cast<uint32_t>(RegCR1::ACTIVE)) SPIx->CR1 &= ~static_cast<uint32_t>(RegCR1::ACTIVE);

    SPIx->CR1 = (static_cast<uint32_t>(type) | static_cast<uint32_t>(wordsize));
    SPIx->CR1 |= static_cast<uint32_t>(RegCR1::ACTIVE);

    RegisterCR1 = SPIx->CR1;
    RegisterCR2 = SPIx->CR2;
    return;
}


void SPI::SetRegisterCR2(RegCR1 MS, RegCR1 Test, RegCR1 SOD)
{
    if (SPIx->CR1 & static_cast<uint32_t>(RegCR1::ACTIVE)) SPIx->CR1 &= ~static_cast<uint32_t>(RegCR1::ACTIVE);

    SPIx->CR1 |= (static_cast<uint32_t>(SOD) | static_cast<uint32_t>(MS) | static_cast<uint32_t>(Test));

    SPIx->CR1 |= static_cast<uint32_t>(RegCR1::ACTIVE);
    RegisterCR1 = SPIx->CR1;
    RegisterCR2 = SPIx->CR2;
    return;
}

/**************************************************************************************************
 * @brief Регистр управления тактовой частотой SPI
 * Включает тактирование модуля SPIx
 * Устанавливает делитель тактовой частоты SPIx
 * Расчитывает частоту тактирования модуля SPIx
 * @param divider Делитель тактовой частоты SPIx
 * @return double частота тактирования модуля SPIx
 *************************************************************************************************/
bool SPI::ClockSettingSPI()
{
    if (SPIx == SPI1)
    {
        F_SPICLK = ClockSystem::APB2BusClock;
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        //  if ((_pinMOSI->_GPIOx == GPIOA) && (_pinMISO->_GPIOx == GPIOA) &&
        //      (_pinSCLK->_GPIOx == GPIOA))
        //      AFIO->MAPR &= ~AFIO_MAPR_SPI1_REMAP;
        //  else if ((_pinMOSI->_GPIOx == GPIOB) && (_pinMISO->_GPIOx == GPIOB) &&
        //           (_pinSCLK->_GPIOx == GPIOB))
        //      AFIO->MAPR |= AFIO_MAPR_SPI1_REMAP;
        //  return true;
    }
    if (SPIx == SPI2)
    {
        F_SPICLK = ClockSystem::APB1BusClock;
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        return true;
    }
    if (SPIx == SPI3)
    {
        F_SPICLK = ClockSystem::APB2BusClock;
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        return true;
    }
    return false;
}

bool SPI::ChipSelect(LVL level)
{
    if (!_pinNSS) return false;
    if (static_cast<LVL>(_pinNSS->GetPinLevel()) == level) return true;
    if (!HardwareSettings()) return false;

    _pinNSS->SetPinLevel(level);
    ProgrammTimer timeout(0.1);
    while ((_pinNSS->GetPinLevel() != static_cast<bool>(level)) && !timeout.TimeOutEvent()) {}

    // if (timeout.TimeOutEvent()) MESSAGE_TRACE();


    return true;
}
