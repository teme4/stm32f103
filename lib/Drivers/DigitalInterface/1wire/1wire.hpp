
#ifndef __ONE_WIRE_DRIVER_HPP__
#define __ONE_WIRE_DRIVER_HPP__

#include "stm32f1xx.h"
#include <cmath>
#include <array>
#include <vector>
#include <memory>
#include "../abstract_interface.hpp"
#include <PortDriver/pin.hpp>
#include <Time&Sync/drivers.hpp>

class OneWire
{
  public:
    OneWire(std::unique_ptr<PINx> pinWire) : _pinWire(std::move(_pinWire)) {}
    OneWire()                           = default;
    OneWire(OneWire const &)            = default;
    OneWire(OneWire &&)                 = default;
    OneWire &operator=(OneWire const &) = default;
    OneWire &operator=(OneWire &&)      = default;
    ~OneWire(){};

  private:
    std::unique_ptr<PINx> _pinWire;
};


#endif//__ONE_WIRE_DRIVER_HPP__
