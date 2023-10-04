/**************************************************************************************************
 * @file spi.hpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.0
 * @date 30.06.2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/
#ifndef _SPI_HPP
#define _SPI_HPP

#include <stm32f1xx.h>
#include <cmath>
#include <array>
#include <vector>
#include <memory>
#include "../abstract_interface.hpp"
#include <PortDriver/pin.hpp>
#include <Time&Sync/drivers.hpp>

struct SPILines
{
    SPI_TypeDef *SPIx;
    PINx MOSI;
    PINx MISO;
    PINx SCLK;
    PINx NSS;
};

extern uint32_t SystemCoreClock;
// clang-format off
enum class IRQnSPI
{
    TXEIE                                       = (1<<7),///<прерываниe по переполнени. приемного буфера FIFO
    RXNEIE                                      = (1<<6),///<прерываниe по таймауту приемника (буфер FIFO приемника не пуст и не было попыток его чтения в течение времени таймаута)
    ERRIE                                       = (1<<5),///<прерываниe по заполнению на 50 % и более буфера FIFO приемника
    NONE                                        = (0<<0)
};

enum class RegCR1
{
    SPI_MODE0                                   = (0b00 ),///<SPI фирмы Motorola(CPOL = 0, CPHA = 0);
    SPI_MODE1                                   = (0b01 ),///<SPI фирмы Motorola(CPOL = 0, CPHA = 1);
    SPI_MODE2                                   = (0b10 ),///<SPI фирмы Motorola(CPOL = 1, CPHA = 0);
    SPI_MODE3                                   = (0b11 ),///<SPI фирмы Motorola(CPOL = 1, CPHA = 1);
    MASTER                                      = (1<<2 ),///<ведущий модуль
    SLAVE                                       = (0<<0 ),///<ведомый модуль
    ACTIVE                                      = (1<<6 ),///<работа разрешена
    INACTIVE                                    = (0<<0 ),///<работа запрещена;
    DFF8bit                                     = (0<<0 ),///<data frame format: 8bit;
    DFF16bit                                    = (1<<11),///<data frame format: 16bit;
    LSBF                                        = (1<<7 ),///<MSB transmitted first
    MSBF                                        = (0<<0 ),///<LSB transmitted first
};
enum class RegDMACR
{
    RXDMA_DIS                                   = 0x00,///<Запрещено формирование запросов DMA буфера FIFO приемника
    RXDMA_EN                                    = 0x01,///<Разрешено формирование запросов DMA буфера FIFO приемника
    TXDMA_DIS                                   = 0x00,///<Запрещено формирование запросов DMA буфера FIFO передатчика
    TXDMA_EN                                    = 0x02,///<Разрешено формирование запросов DMA буфера FIFO передатчика
    NONE                                        = 0x00
};
// clang-format on

/**************************************************************************************************
 * @brief Construct a new SPI::SPI object
 *
 * @param SPI MDR_SPIx
 * @param sse Разрешение работы приемопередатчика
 * @param ms Выбор ведущего или ведомого режима работы
 * @param frequency speed Mbps
 * @param type Формат информационного кадра (+режим SPI)
 *           @arg SPI_MODEx
 *           @arg SSI_TI
 *           @arg MicroWire
 * @param wordsize Размер слова данных
 * @param test Тестирование по шлейфу
 * @param sod Запрет выходных линий в режиме ведомого устройства
 * @param irq Список событий на прерывание
 * @param txdmacr Использование DMA передатчика
 * @param rxdmacr Использование DMA приемника
 *************************************************************************************************/

class SPI : public DigitalInterface
{
  public:
    bool SettingsAccident{true};

    /**************************************************************************************************
     * @brief Construct a new SPI object
     *
     * @param SPI адрес модуля SPIx
     * @param pins asdfaf
     *************************************************************************************************/
    explicit SPI(SPI_TypeDef *CurrentSPI, std::array<std::unique_ptr<PINx>, 4> &&pins) :
            SPIx(CurrentSPI),
            _pinMOSI(std::move(pins.at(0))),
            _pinMISO(std::move(pins.at(1))),
            _pinSCLK(std::move(pins.at(2))),
            _pinNSS(std::move(pins.at(3)))
    {
        if (!ClockSettingSPI()) return;
        HardwareSettings();
    }

    /**************************************************************************************************
     * @brief Construct a new SPI object
     *
     * @param SPI адрес модуля SPIx
     * @param pins asdfaf
     *************************************************************************************************/
    explicit SPI(SPILines baremetal) :
            SPIx(baremetal.SPIx),
            _pinMOSI(std::make_unique<PINx>(baremetal.MOSI)),
            _pinMISO(std::make_unique<PINx>(baremetal.MISO)),
            _pinSCLK(std::make_unique<PINx>(baremetal.SCLK)),
            _pinNSS(std::make_unique<PINx>(baremetal.NSS))
    {
        HardwareSettings();
    }
    SPI()                       = default;
    SPI(SPI const &)            = default;
    SPI(SPI &&)                 = default;
    SPI &operator=(SPI const &) = default;
    SPI &operator=(SPI &&)      = default;
    ~SPI()                      = default;

    bool HardwareSettings();
    virtual uint32_t GetAddressRegisterDR() override { return (reinterpret_cast<uint32_t>(&(SPIx->DR))); }
    virtual bool CheckReadiness() override { return !SettingsAccident; }
    virtual std::vector<uint8_t> Transmitt(std::vector<uint8_t> &packet) override;
    virtual std::vector<uint8_t> Recieve(std::vector<uint8_t> &packet) override;


    void SettingsSPI(RegCR1 SPE,
                     RegCR1 MS,
                     double frequency,
                     RegCR1 Type,
                     RegCR1 WordSize,
                     RegCR1 LsbMsbFirst,
                     std::vector<IRQnSPI> irq,
                     RegDMACR TxDmacr,
                     RegDMACR RxDmacr);

    void SetRegisterCR1(RegCR1 type, RegCR1 wordsize);
    void SetRegisterCR2(RegCR1 ms, RegCR1 test, RegCR1 sod);
    void SPIxEnable(RegCR1 sse);

  private:
    SPI_TypeDef *SPIx;
    uint16_t RegisterCR1{0};
    uint16_t RegisterCR2{0};
    uint8_t RegisterCPSR{0};
    uint8_t RegisterRXCRCR{0};
    uint8_t RegisterTXCRCR{0};
    double F_SPICLK{0};
    double Frequency{0};
    uint8_t BRR{0};

    std::unique_ptr<PINx> _pinMOSI;
    std::unique_ptr<PINx> _pinMISO;
    std::unique_ptr<PINx> _pinSCLK;
    std::unique_ptr<PINx> _pinNSS;
    bool CheckSettings();
    bool ChipSelect(LVL select);
    bool ClockSettingSPI();
    void SetFrequency(double freq);
};

#endif//_SPI_HPP
