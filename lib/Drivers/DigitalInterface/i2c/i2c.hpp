/**************************************************************************************************
 * @file i2c.hpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.0
 * @date 22.12.2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/
#ifndef __I2C_HARDWARE_DRIVER_HPP__
#define __I2C_HARDWARE_DRIVER_HPP__


#include "stm32f1xx.h"
#include <vector>
#include <memory>
#include "../abstract_interface.hpp"
#include <PortDriver/pin.hpp>
#include <Time&Sync/drivers.hpp>


struct I2CLines
{
    I2C_TypeDef *I2Cx;
    PINx SDA;
    PINx SCL;
};

class I2C : public DigitalInterface
{
  public:
    double Frequency;
    bool SettingsAccident{true};

    explicit I2C(I2C_TypeDef *i2c, double freq, double timeout, std::unique_ptr<PINx> sda, std::unique_ptr<PINx> scl) :
            Frequency(freq),
            _TimeOut(timeout),
            _I2Cx(i2c),
            _pinSDA(std::move(sda)),
            _pinSCL(std::move(scl))
    {
        _HardwareSettings();
    }
    I2C()                       = default;
    I2C(I2C const &)            = default;
    I2C(I2C &&)                 = default;
    I2C &operator=(I2C const &) = default;
    I2C &operator=(I2C &&)      = default;
    ~I2C(){};

    virtual bool CheckReadiness() override { return !SettingsAccident; }
    virtual uint32_t GetAddressRegisterDR() override { return (reinterpret_cast<uint32_t>(&(_I2Cx->DR))); }
    virtual std::vector<uint8_t> Transmitt(std::vector<uint8_t> &packet) override;
    virtual std::vector<uint8_t> Recieve(std::vector<uint8_t> &packet) override;


  private:
    double _TimeOut;
    I2C_TypeDef *_I2Cx;
    std::unique_ptr<PINx> _pinSDA{};
    std::unique_ptr<PINx> _pinSCL{};

    void _TrubleHandler();
    //#FIXME: uint16_t GetPrescaller(double frequency){return static_cast<uint16_t>(TSCL / (2 * TPCLK1));};
    bool _HardwareSettings();
};


#endif//__I2C_HARDWARE_DRIVER_HPP__
